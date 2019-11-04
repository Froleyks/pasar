#include "greedy_best_first.hpp"

#include "src/learn_actions.hpp"

#include <set>

// public
GreedyBestFirst::GreedyBestFirst(Problem &problem)
    : BaseSearch(problem), state(problem.initialState), compactState(state) {
  computeActionSupport();
  reset({problem.goalState});
}

inline void GreedyBestFirst::updateGuideState(const std::vector<State> &guideStates,
                                              std::vector<action_t> &plan) {
  // ignore actual goal state
  if (guideStates.size() < 2) {
    return;
  }
  for (size_t s = firstGuideState; s < guideStates.size() - 1; ++s) {
    if (!reachedGuideHint[s]) {
      continue;
    }
    bool satisfied = true;
    for (variable_t variable = 0; variable < state.size(); ++variable) {
      if (guideStates[s][variable] == unassigned) {
        continue;
      }
      if (guideStates[s][variable] != state[variable]) {
        satisfied = false;
        break;
      }
    }

    if (satisfied) {
      // learn action to jump from last found guide state to this one
      action_t a = addActionToProblem(problem_, plan, milestones.back().second,
                                      plan.size() - 1);
      if (milestones.back().second < plan.size() - 1) {
        numLearnedActions++;
        sumSkipped += plan.size() - milestones.back().second;
        LOG(4) << "added state space search action " << a << " "
               << milestones.back().second << " " << plan.size() - 1;
      }
      milestones.emplace_back(firstGuideState, plan.size());
      LOG(4) << "update guide state " << firstGuideState << " to " << s + 1;
      firstGuideState = s + 1;
    }
  }
}

inline bool GreedyBestFirst::continueSearch(const std::vector<State> &guideStates,
                                     std::vector<action_t> &plan) {
  LOG(4) << "start forward search";

  double endTime    = Logger::getTime() + searchTimeout_;
  int exploredNodes = 0;

  while (!assignmentHolds(problem_.goal, state)) {
    if (exploredNodes++ % 128 == 0 &&
        ((searchLimit_ > -1 && exploredNodes > searchLimit_) ||
         Logger::getTime() > endTime)) {
      return false;
    }
    if (appActions.empty()) {
      // deadend -> backtrack
      if (changeHistory.empty()) {
        LOG(4) << "no actions applicable -> Problem unsolveable";
        return false;
      }
      applyAssignment(changeHistory.back(), compactState);
      applyAssignment(changeHistory.back(), state);
      updateApplicableActions(state, changeHistory.back(), guideStates,
                              appActions);
      changeHistory.pop_back();
      // action that will be removed is a milestone
      if (plan.size() < milestones.back().second) {
        firstGuideState = milestones.back().first;
        milestones.pop_back();
      }
      reachedGuideHint.resize(guideStates.size(), false);
      plan.pop_back();
    } else {
      bool appliedAction = false;
      for (auto [gain, a] : appActions) {
        Assignment oldValues;
        applyAssignment(problem_.eff[a], compactState, oldValues);
        bool newState = knownStates.insert(compactState).second;
        if (!newState) {
          // state was visited before
          applyAssignment(oldValues, compactState);
        } else {
          plan.push_back(a);
          changeHistory.push_back(oldValues);
          applyAssignment(problem_.eff[a], state);
          updateGuideState(guideStates, plan);
          updateApplicableActions(state, problem_.eff[a], guideStates,
                                  appActions, reachedGuideHint);
          appliedAction = true;
          break;
        }
      }
      if (!appliedAction) {
        // all child states where visited -> backtrack
        if (changeHistory.empty()) {
          LOG(4) << "exhausted search space";
          return false;
        }
        applyAssignment(changeHistory.back(), compactState);
        applyAssignment(changeHistory.back(), state);
        updateApplicableActions(state, changeHistory.back(), guideStates,
                                appActions);
        changeHistory.pop_back();
        // action that will be removed is a milestone
        if (plan.size() < milestones.back().second) {
          firstGuideState = milestones.back().first;
          milestones.pop_back();
        }
        reachedGuideHint.resize(guideStates.size(), false);
        plan.pop_back();
      }
    }
  }
  return true;
}

inline void GreedyBestFirst::reset(const std::vector<State> &guideStates) {
  firstGuideState = 0;
  state           = problem_.initialState;
  compactState    = state;
  knownStates     = {compactState};
  updateActionSupport();
  getApplicableActions(state, guideStates, appActions);

  // before plan index <scond> <first> was the highest reached guideStates
  milestones = {{-1, 0}};
  milestones.reserve(guideStates.size());

  // used to indicate if there is an action in the set of applicable actions
  // that might reach this state
  reachedGuideHint.resize(guideStates.size(), false);
}

bool GreedyBestFirst::search(std::vector<State> &guideStates,
                             std::vector<action_t> &plan) {
  if (searchTimeout_ == 0 || searchLimit_ == 0) {
    return false;
  }
  reset(guideStates);

  bool solved = continueSearch(guideStates, plan);
  if (!solved) {
    plan.clear();
  }
  return solved;
}
