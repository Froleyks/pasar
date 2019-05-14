#include "depth_first_search.hpp"

#include "state_hash.hpp"

// public
DepthFirstSearch::DepthFirstSearch(Problem &problem) : BaseSearch(problem) {
  computeActionSupport();
}

bool DepthFirstSearch::search(std::vector<State> &guideStates,
                              std::vector<action_t> &plan, double timeLimit) {
  log(3) << "start forward search";

  double endTime = Logger::getTime() + timeLimit;

  State state = problem.initialState;
  CompactState compactState(state);
  StateHashSet knownStates{compactState};

  std::vector<Assignment> changeHistory;

  computeActionSupport();
  std::vector<action_t> appActions;
  getApplicableActions(state, appActions);

  size_t checkTime = 0;
  while (!assignmentHolds(problem.goal, state)) {
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
      updateApplicableActions(state, changeHistory.back(), appActions);
      changeHistory.pop_back();
      plan.pop_back();
    } else {
      bool appliedAction = false;
      for (auto a : appActions) {
        Assignment oldValues;
        applyAssignment(problem.eff[a], compactState, oldValues);
        bool newState = knownStates.insert(compactState).second;
        if (!newState) {
          // state was visited before
          applyAssignment(oldValues, compactState);
        } else {
          plan.push_back(a);
          changeHistory.push_back(oldValues);
          applyAssignment(problem.eff[a], state);
          updateApplicableActions(state, problem.eff[a], appActions);
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
        updateApplicableActions(state, changeHistory.back(), appActions);
        changeHistory.pop_back();
        plan.pop_back();
      }
    }
  }
  return true;
}
