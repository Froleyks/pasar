#include "greedy_best_first.hpp"

#include <set>

// private
inline void GreedyBestFirst::assignGain(State &state,
                                        std::vector<State> &guideStates,
                                        std::vector<action_t> &appActions,
                                        std::vector<double> &weights) {
  weights.clear();
  weights.reserve(appActions.size());
  for (int a = 0; a < appActions.size(); ++a) {
    weights.emplace_back(
        gainOfAssignment(problem.eff[appActions[a]], state, guideStates));
  }
}

// public
GreedyBestFirst::GreedyBestFirst(Problem &problem) : BaseSearch(problem) {
  computeActionSupport();
}

bool GreedyBestFirst::search(std::vector<State> &guideStates,
                             std::vector<action_t> &plan, double timeLimit) {
  log(3) << "start forward search";

  double endTime = Logger::getTime() + timeLimit;

  State state = problem.initialState;
  CompactState compactState(state);
  StateHashSet knownStates{compactState};

  std::vector<Assignment> changeHistory;

  updateActionSupport();
  WeightedActionSet appActions;
  getApplicableActions(state, guideStates, appActions);

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
      updateApplicableActions(state, changeHistory.back(), guideStates,
                              appActions);
      changeHistory.pop_back();
      plan.pop_back();
    } else {
      bool appliedAction = false;
      for (auto [gain, a] : appActions) {
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
          updateApplicableActions(state, problem.eff[a], guideStates,
                                  appActions);
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
        plan.pop_back();
      }
    }
  }
  return true;
}

size_t GreedyBestFirst::lastGuideState() { return lastGuide; }
