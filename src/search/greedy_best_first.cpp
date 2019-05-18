#include "greedy_best_first.hpp"

#include "src/learn_actions.hpp"

#include <set>

// public
GreedyBestFirst::GreedyBestFirst(Problem &problem) : BaseSearch(problem) {
  computeActionSupport();
}

inline void GreedyBestFirst::updateGuideState(
    State &state, std::vector<State> &guideStates, std::vector<action_t> &plan,
    std::vector<std::pair<size_t, size_t>> &milestones,
    std::vector<char> &reachedGuideHint) {
  // ignore actual goal state
  if (guideStates.size() < 2) {
    return;
  }
  for (size_t s = guideStates.size() - 2;
       s != firstGuideState - 1; --s) {
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
      log(4) << "added state space search action " << a;
      milestones.emplace_back(firstGuideState, plan.size());
      firstGuideState = s + 1;
    }
  }
}

bool GreedyBestFirst::search(std::vector<State> &guideStates,
                             std::vector<action_t> &plan, double timeLimit) {
  log(3) << "start forward search";

  double endTime = Logger::getTime() + timeLimit;

  State state = problem_.initialState;
  CompactState compactState(state);
  StateHashSet knownStates{compactState};

  std::vector<Assignment> changeHistory;

  updateActionSupport();
  WeightedActionSet appActions;
  getApplicableActions(state, guideStates, appActions);

  // encountered guideState <first> at plan index <second>
  std::vector<std::pair<size_t, size_t>> milestones;
  milestones.reserve(guideStates.size());

  size_t checkTime = 0;
  // used to indicate if there is an action in the set of applicable actions
  // that might reach this state
  std::vector<char> reachedGuideHint(guideStates.size(), false);
  while (!assignmentHolds(problem_.goal, state)) {
    if (checkTime++ % 128 == 0 && Logger::getTime() > endTime) {
      plan.clear();
      return false;
    }
    if (appActions.empty()) {
      // deadend -> backtrack
      if (changeHistory.empty()) {
        plan.clear();
        log(4) << "no actions applicable -> Problem unsolveable";
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
          updateGuideState(state, guideStates, plan, milestones,
                           reachedGuideHint);
          updateApplicableActions(state, problem_.eff[a], guideStates,
                                  appActions, reachedGuideHint);
          appliedAction = true;
          break;
        }
      }
      if (!appliedAction) {
        // all child states where visited -> backtrack
        if (changeHistory.empty()) {
          plan.clear();
          log(4) << "exhausted search space";
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
