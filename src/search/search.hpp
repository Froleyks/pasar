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
    int gain;
    int action;
    WeightedAction(int gain, int action) : gain(gain), action(action) {}
  };

  struct CompWeightedActions {
    bool operator()(const WeightedAction &a, const WeightedAction &b) const {
      // they are equivalent if !comp(a,b) && !comp(b,a)
      if (a.action == b.action) {
        return false;
      }
      return a.gain >= b.gain;
    }
  };

  using WeightedActionSet = std::set<WeightedAction, CompWeightedActions>;

  Problem &problem;

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
                                 std::vector<State> &guideStates,
                                 const bool randomize = true) {
    double gain  = 0;
    double decay = 1;
    // start from actual goal
    for (int s = guideStates.size() - 1; s > -1; --s) {
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

    if (randomize) {
      // random tie-breaking
      gain *= noiseRange;
      const double noise = ((double)rand() * noiseRange) / RAND_MAX;
      gain += noise;
    }

    return gain;
  }

  inline void computeActionSupport() {
    actionSupport.resize(problem.numVariables);
    for (int v = 0; v < problem.numValues.size(); ++v) {
      actionSupport[v].resize(problem.numValues[v]);
    }
    for (int v = problem.numValues.size(); v < problem.numVariables; ++v) {
      actionSupport[v].resize(2);
    }
    for (size_t a = 0; a < problem.pre.size(); ++a) {
      for (auto [variable, value] : problem.pre[a]) {
        actionSupport[variable][value].push_back(a);
      }
    }
    firstNewAction = problem.numActions;
  }

  inline void updateActionSupport() {
    for (size_t a = firstNewAction; a < problem.pre.size(); ++a) {
      for (auto [variable, value] : problem.pre[a]) {
        actionSupport[variable][value].push_back(a);
      }
    }
    firstNewAction = problem.numActions;
  }

  inline void getApplicableActions(const State &state,
                                   std::vector<action_t> &appActions) {
    appActions.clear();
    for (size_t a = 0; a < problem.pre.size(); ++a) {
      if (assignmentHolds(problem.pre[a], state)) {
        appActions.push_back(a);
      }
    }
  }

  inline void getApplicableActions(const State &state,
                                   std::vector<State> &guideStates,
                                   WeightedActionSet &appActions) {
    appActions.clear();
    for (action_t a = 0; a < problem.numActions; ++a) {
      if (assignmentHolds(problem.pre[a], state)) {
        appActions.emplace(gainOfAssignment(problem.eff[a], state, guideStates),
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
      if (assignmentHolds(problem.pre[action], state)) {
        appActions.emplace_back(action);
      }
    }

    // actions that might become available
    for (auto [variable, value] : newValues) {
      // all actions which have variable == value as a precondition
      for (auto a : actionSupport[variable][value]) {
        if (assignmentHolds(problem.pre[a], state)) {
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
      if (assignmentHolds(problem.pre[action], state)) {
        appActions.emplace(
            gainOfAssignment(problem.eff[action], state, guideStates), action);
      }
    }

    // actions that might become available
    for (auto [variable, value] : newValues) {
      // all actions which have variable == value as a precondition
      for (auto a : actionSupport[variable][value]) {
        if (assignmentHolds(problem.pre[a], state)) {
          appActions.emplace(
              gainOfAssignment(problem.eff[a], state, guideStates), a);
        }
      }
    }
  }

public:
  BaseSearch(Problem &problem) : problem(problem) {
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
};
