#include "greedy_best_first.hpp"

#include "src/learn_actions.hpp"

#include <set>

// public
GreedyBestFirst::GreedyBestFirst(Problem &problem) : BaseSearch(problem) {
  computeActionSupport();
}

inline void GreedyBestFirst::updateGuideState(
    State &state, std::vector<State> &guideStates, std::vector<action_t> &plan,
    std::vector<std::pair<int, size_t>> &milestones) {
  for (int s = guideStates.size() - 2; s >= (int)firstGuideState; --s) {
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
      action_t a = addActionToProblem(problem, plan, milestones.back().second,
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

  State state = problem.initialState;
  CompactState compactState(state);
  StateHashSet knownStates{compactState};

  std::vector<Assignment> changeHistory;

  updateActionSupport();
  WeightedActionSet appActions;
  getApplicableActions(state, guideStates, appActions);

  // encountered guideState <first> at plan index <second>
  std::vector<std::pair<int, size_t>> milestones = {{-1, 0}};
  milestones.reserve(guideStates.size());

  size_t checkTime      = 0;
  bool hintReachedGuide = false;
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
      // action that will be removed is a milestone
      if (plan.size() < milestones.back().second) {
        firstGuideState = milestones.back().first;
        milestones.pop_back();
      }
      hintReachedGuide = false;
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
          if (hintReachedGuide) {
            updateGuideState(state, guideStates, plan, milestones);
          }
          hintReachedGuide = updateApplicableActions(state, problem.eff[a],
                                                     guideStates, appActions);
          appliedAction    = true;
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
        hintReachedGuide = false;
        plan.pop_back();
      }
    }
  }
  return true;
}
