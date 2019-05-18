#pragma once

#include <set>
#include <vector>

#include "src/problem/abstract_plan.hpp"
#include "src/problem/problem.hpp"
#include "src/search/state_hash.hpp"
#include "src/type_defs.hpp"

class BaseSearch {
private:
protected:
  // sort actions by gain
  struct WeightedAction {
    // TODO this should be a float
    int gain_;
    action_t action_;
    WeightedAction(int gain, action_t action) : gain_(gain), action_(action) {}
  };

  struct CompWeightedActions {
    bool operator()(const WeightedAction &a, const WeightedAction &b) const {
      // they are equivalent if !comp(a,b) && !comp(b,a)
      if (a.action_ == b.action_) {
        return false;
      }
      return a.gain_ >= b.gain_;
    }
  };

  using WeightedActionSet = std::set<WeightedAction, CompWeightedActions>;

  Problem &problem_;

  // used for random tie-breaking
  static constexpr int noiseRange = 10;
  // gainDecayFactor ∈ (0, 1]
  // low gainDecayFactor ⇒ guideStates further from the goal have less  impact
  // on the direction of the search
  static constexpr double gainDecayFactor = 0.8;

  // (variable, value)s ↦ actions; v → a ⇔ v ∈ pre(a)
  std::vector<std::vector<std::vector<action_t>>> actionSupport;

  // the first actions that was added after the last updateActionSupport
  action_t firstNewAction = 0;

  // update a compact representation of the state used for hashing
  inline void applyAssignment(const Assignment &a, CompactState &compactState) {
    for (auto [variable, value] : a) {
      compactState.set(variable, value);
    }
  }

  // save the overwritten values
  inline void applyAssignment(const Assignment &a, CompactState &compactState,
                              Assignment &oldValues) {
    oldValues.clear();
    oldValues.reserve(a.size());
    for (auto [variable, value] : a) {
      oldValues.emplace_back(variable, compactState.set(variable, value));
    }
  }

  inline double gainOfAssignment(const Assignment &a, const State &state,
                                 const std::vector<State> &guideStates,
                                 std::vector<char> &reachedGuideHint) {
    double gain  = 0;
    double decay = 1;
    // start from actual goal
    for (size_t s = guideStates.size() - 1; s != firstGuideState - 1; --s) {
      bool mightReacheGuide = true;
      // compute change in hamming distance
      int stepGain = 0;
      for (auto [variable, value] : a) {
        if (guideStates[s][variable] == unassigned) {
          continue;
        }
        if (guideStates[s][variable] == value) {
          if (state[variable] != value) {
            stepGain += 1;
          }
        } else {
          if (state[variable] == guideStates[s][variable]) {
            mightReacheGuide = false;
            stepGain -= 1;
          }
        }
      }
      mightReacheGuide = mightReacheGuide && stepGain;
      reachedGuideHint[s] |= static_cast<char>(mightReacheGuide);
      gain += stepGain * decay;
      decay *= gainDecayFactor;
    }

    // random tie-breaking
    gain *= noiseRange;
    const double noise = (static_cast<double>(rand()) * noiseRange) / RAND_MAX;
    gain += noise;

    return gain;
  }

  inline double gainOfAssignment(const Assignment &a, const State &state,
                                 std::vector<State> &guideStates) {
    double gain  = 0;
    double decay = 1;
    // start from actual goal
    for (size_t s = guideStates.size() - 1; s != static_cast<size_t>(-1); --s) {
      // compute change in hamming distance
      int stepGain = 0;
      for (auto [variable, value] : a) {
        if (guideStates[s][variable] == unassigned) {

          continue;
        }
        if (guideStates[s][variable] == value) {
          if (state[variable] != value) {
            stepGain += 1;
          }
        } else {
          if (state[variable] == guideStates[s][variable]) {
            stepGain -= 1;
          }
        }
      }
      gain += stepGain * decay;
      decay *= gainDecayFactor;
    }

    // random tie-breaking
    gain *= noiseRange;
    const double noise = (static_cast<double>(rand()) * noiseRange) / RAND_MAX;
    gain += noise;

    return gain;
  }

  inline void computeActionSupport() {
    actionSupport.resize(problem_.numVariables);
    for (size_t v = 0; v < problem_.numValues.size(); ++v) {
      actionSupport[v].resize(problem_.numValues[v]);
    }
    for (size_t v = problem_.numValues.size(); v < problem_.numVariables; ++v) {
      actionSupport[v].resize(2);
    }
    for (size_t a = 0; a < problem_.pre.size(); ++a) {
      for (auto [variable, value] : problem_.pre[a]) {
        actionSupport[variable][value].push_back(static_cast<action_t>(a));
      }
    }
    firstNewAction = static_cast<action_t>(problem_.numActions);
  }

  inline void updateActionSupport() {
    for (size_t a = firstNewAction; a < problem_.pre.size(); ++a) {
      for (auto [variable, value] : problem_.pre[a]) {
        actionSupport[variable][value].push_back(static_cast<action_t>(a));
      }
    }
    firstNewAction = static_cast<action_t>(problem_.numActions);
  }

  // used only once
  inline void getApplicableActions(const State &state,
                                   std::vector<action_t> &appActions) {
    appActions.clear();
    for (size_t a = 0; a < problem_.pre.size(); ++a) {
      if (assignmentHolds(problem_.pre[a], state)) {
        appActions.push_back(static_cast<action_t>(a));
      }
    }
  }

  inline void getApplicableActions(const State &state,
                                   std::vector<State> &guideStates,
                                   WeightedActionSet &appActions) {
    appActions.clear();
    for (action_t a = 0; a < problem_.numActions; ++a) {
      if (assignmentHolds(problem_.pre[a], state)) {
        appActions.emplace(gainOfAssignment(problem_.eff[a], state, guideStates),
                           a);
      }
    }
  }

  inline void updateApplicableActions(const State &state,
                                      const Assignment &newValues,
                                      std::vector<action_t> &appActions) {
    // remove no longer applicable actions
    std::vector<action_t> oldAppActions = std::move(appActions);
    appActions.clear();
    for (auto action : oldAppActions) {
      if (assignmentHolds(problem_.pre[action], state)) {
        appActions.emplace_back(action);
      }
    }

    // actions that might become available
    for (auto [variable, value] : newValues) {
      // all actions which have variable == value as a precondition
      for (auto a : actionSupport[variable][value]) {
        if (assignmentHolds(problem_.pre[a], state)) {
          appActions.push_back(a);
        }
      }
    }
  }

  inline void updateApplicableActions(const State &state,
                                      const Assignment &newValues,
                                      std::vector<State> &guideStates,
                                      WeightedActionSet &appActions) {
    // remove no longer applicable actions
    WeightedActionSet oldAppActions = std::move(appActions);
    appActions.clear();
    for (auto [gain, action] : oldAppActions) {
      if (assignmentHolds(problem_.pre[action], state)) {
        appActions.emplace(
            gainOfAssignment(problem_.eff[action], state, guideStates), action);
      }
    }

    // actions that might become available
    for (auto [variable, value] : newValues) {
      // all actions which have variable == value as a precondition
      for (auto a : actionSupport[variable][value]) {
        if (assignmentHolds(problem_.pre[a], state)) {
          appActions.emplace(
              gainOfAssignment(problem_.eff[a], state, guideStates), a);
        }
      }
    }
  }

  // return a hint if a guide state could be reached
  inline void updateApplicableActions(const State &state,
                                      const Assignment &newValues,
                                      std::vector<State> &guideStates,
                                      WeightedActionSet &appActions,
                                      std::vector<char> &reachedGuideHint) {
    reachedGuideHint.resize(guideStates.size(), false);
    // remove no longer applicable actions
    WeightedActionSet oldAppActions = std::move(appActions);
    appActions.clear();
    for (auto [gain, action] : oldAppActions) {
      if (assignmentHolds(problem_.pre[action], state)) {
        appActions.emplace(gainOfAssignment(problem_.eff[action], state,
                                            guideStates, reachedGuideHint),
                           action);
      }
    }

    // actions that might become available
    for (auto [variable, value] : newValues) {
      // all actions which have variable == value as a precondition
      for (auto a : actionSupport[variable][value]) {
        if (assignmentHolds(problem_.pre[a], state)) {
          appActions.emplace(gainOfAssignment(problem_.eff[a], state,
                                              guideStates, reachedGuideHint),
                             a);
        }
      }
    }
  }

public:
  BaseSearch(Problem &problem) : problem_(problem) {
    constexpr unsigned seed = 42;
    srand(seed);
  }

  // check preconditions and goal
  static inline bool assignmentHolds(const Assignment &a, const State &state) {
    for (auto [variable, value] : a) {
      if (state[variable] != value) {
        return false;
      }
    }
    return true;
  }

  // apply action effects and restore old values
  static inline void applyAssignment(const Assignment &a, State &state) {
    for (auto [variable, value] : a) {
      state[variable] = value;
    }
  }
  size_t firstGuideState = 0;
};
