#pragma once

#include "src/learn_actions.hpp"
#include "src/problem/problem.hpp"
#include "src/search/search.hpp"
#include "src/type_defs.hpp"
#include "tools/parameter_processor.hpp"

#include "src/search/depth_first_search.hpp"

#include "abstraction/cegar_foreach.hpp"

class ADAL {
private:
  // parameters
  int sparsification        = 1;
  bool contraction          = false;
  double abstractionTimeout = std::numeric_limits<double>::infinity();
  double searchTimeout      = std::numeric_limits<double>::infinity();

  Problem &problem;

  void abstractActionElimination(std::vector<AbstractPlan::Step> &steps) {
    State state = problem.initialState;
    for (int i = 0; i < (int)steps.size(); ++i) {
      State endPlanState = state;
      std::vector<AbstractPlan::Step> endPlan;
      for (size_t j = i + 1; j < steps.size(); ++j) {
        bool parallelApplicable = true;
        for (size_t a = 0; a < steps[j].size(); ++a) {
          if (!BaseSearch::assignmentHolds(problem.pre[steps[j][a]],
                                           endPlanState)) {
            parallelApplicable = false;
          }
        }
        if (parallelApplicable) {
          endPlan.push_back(steps[j]);
          for (size_t a = 0; a < steps[j].size(); ++a) {
            BaseSearch::applyAssignment(problem.eff[steps[j][a]], endPlanState);
          }
        }
      }
      if (BaseSearch::assignmentHolds(problem.goal, endPlanState)) {
        steps.erase(steps.begin() + i, steps.end());
        steps.insert(steps.end(), endPlan.begin(), endPlan.end());
        i--;
      } else {
        for (size_t a = 0; a < steps[i].size(); ++a) {
          BaseSearch::applyAssignment(problem.eff[steps[i][a]], state);
        }
      }
    }
  }

  // used to extract states form an abstract plan
  // the extracted states are consistent since the abstract plan was consistent
  // side effect: the step is sparsified
  inline void unapplyActions(AbstractPlan::Step &step, State &state) {
    // vector of bool
    std::vector<char> keepAction(step.size(), false);
    // remove fulfilled atoms from state
    for (size_t i = 0; i < step.size(); ++i) {
      for (auto [variable, value] : problem.eff[step[i]]) {
        if (state[variable] == value) {
          keepAction[i]   = true;
          state[variable] = unassigned;
        }
      }
    }
    // add preconditions to state
    for (size_t i = 0; i < step.size(); ++i) {
      if (!keepAction[i]) {
        continue;
      }
      for (auto [variable, value] : problem.pre[step[i]]) {
        assert(state[variable] == unassigned || state[variable] == value);
        state[variable] = value;
      }
    }
    size_t newSize = 0;
    for (size_t i = 0; i < keepAction.size(); ++i) {
      newSize += keepAction[i];
    }
    AbstractPlan::Step tmpStep;
    tmpStep.reserve(newSize);
    for (size_t i = 0; i < keepAction.size(); ++i) {
      if (keepAction[i]) {
        tmpStep.push_back(step[i]);
      }
    }
    step = std::move(tmpStep);
  }

  void sparsify(std::vector<AbstractPlan::Step> &stepSequence) {
    // remove actions from the parallel plan that don't have relevant effects
    State state = problem.goalState;
    for (int s = stepSequence.size() - 1; s > -1; --s) {
      unapplyActions(stepSequence[s], state);
    }

    abstractActionElimination(stepSequence);
  }

  void extractCompleteStates(std::vector<State> &states, Formula &f) {
    states.resize(f.state.size());
    for (size_t t = 0; t < f.state.size(); ++t) {
      states[t].resize(f.state[t].size());
      for (int variable = 0; variable < f.state[t].size(); ++variable) {
        states[t][variable] = f.getStateValue(variable, t);
      }
    }
  }

  void extractPartialStates(std::vector<AbstractPlan::Step> &steps,
                            std::vector<State> &states) {
    states.clear();
    states.reserve(steps.size() + 1);
    State state = problem.goalState;
    states.insert(states.begin(), state);
    for (int s = steps.size() - 1; s > -1; --s) {
      unapplyActions(steps[s], state);
      states.insert(states.begin(), state);
    }
  }

  // tries to fix each step of the abstract plan, adds learned actions and may
  // reduce the states
  template <class Abstraction>
  bool fixPlan(Abstraction &abstraction, AbstractPlan &abstractPlan,
               std::vector<action_t> &plan, std::vector<bool> &stepFixed) {
    size_t abstractPlanLength = abstractPlan.steps.size();
    plan.resize(abstractPlanLength, noop);
    stepFixed.resize(abstractPlanLength, false);
    bool fixedPlan = true;
    for (int s = abstractPlanLength - 1; s > -1; --s) {
      // the abstract plan might be fixed
      std::vector<action_t> planForStep;
      int fixed =
          abstraction.fixStep(abstractPlan.states[s], abstractPlan.steps[s],
                              abstractPlan.states[s + 1], planForStep);
      if (fixed) {
        log(5) << "fixed step " << s;
        stepFixed[s] = true;
        if (planForStep.size() < 2) {
          // one or no action
          plan[s] = planForStep[0];
        } else {
          plan[s] = addActionToProblem(problem, planForStep);
        }
      } else {
        fixedPlan = false;
      }
    }
    return fixedPlan;
  }

  void abstractPlanContraction(const std::vector<bool> &stepFixed,
                               const std::vector<action_t> &plan,
                               const AbstractPlan &abstractPlan,
                               std::vector<State> &guideStates,
                               std::vector<size_t> &originalIndex) {
    size_t planLength = stepFixed.size();
    size_t first      = planLength;
    for (int s = 0; s < planLength; ++s) {
      if (stepFixed[s]) {
        if (first == planLength) {
          first = s;
          // don't add the initial state
          if (s > 0) {
            // add the first state of every shortcut
            guideStates.push_back(abstractPlan.states[s]);
            originalIndex.push_back(s);
          }
        }
      } else {
        addActionToProblem(problem, plan, first, s - 1);
        first = planLength;
        // add the last state of every shortcut
        guideStates.push_back(abstractPlan.states[s]);
        originalIndex.push_back(s);
      }
    }
    if (stepFixed.back()) {
      addActionToProblem(problem, plan, first, planLength - 1);
    }
    guideStates.push_back(abstractPlan.states.back());
    originalIndex.push_back(planLength - 1);
  }

public:
  ADAL(Problem &problem) : problem(problem) {}

  template <class Abstraction, class Search>
  bool findPlan(Abstraction &abstraction, Search &search,
                std::vector<action_t> &plan) {
    AbstractPlan abstractPlan;
    bool solvedAbstraction = false;
    if (abstractionTimeout) {
      solvedAbstraction =
          abstraction.solve(abstractPlan.steps, abstractionTimeout);
    }

    std::vector<State> guideStates;
    if (!solvedAbstraction) {
      guideStates = {problem.goalState};
      return search.search(guideStates, plan, searchTimeout);
    }

    switch (sparsification) {
      // complete states (if possible)
    case 0: {
      if constexpr (Abstraction::isSatBased) {
        extractCompleteStates(abstractPlan.states, abstraction.f);
      } else {
        extractPartialStates(abstractPlan.steps, abstractPlan.states);
      }
      break;
    }
      // partial states
    case 1: {
      extractPartialStates(abstractPlan.steps, abstractPlan.states);
      break;
    }
      // action elimination
    case 2: {
      // remove actions from the parallel plan that don't have relevant effects
      State state = problem.goalState;
      for (int s = abstractPlan.steps.size() - 1; s > -1; --s) {
        unapplyActions(abstractPlan.steps[s], state);
      }
      abstractActionElimination(abstractPlan.steps);
      extractPartialStates(abstractPlan.steps, abstractPlan.states);
      break;
    }
    }

    // try to fix the plan and add actions
    std::vector<bool> stepFixed;
    bool solved = fixPlan(abstraction, abstractPlan, plan, stepFixed);
    if (solved) {
      log(3) << "fixed abstract plan";
      return true;
    }

    /// plan contraction
    // used to find the last visited guide state
    std::vector<size_t> originalIndex;
    if (contraction) {
      abstractPlanContraction(stepFixed, plan, abstractPlan, guideStates,
                              originalIndex);
    } else {
      // ignore first
      guideStates = std::vector<State>(abstractPlan.states.begin() + 1,
                                       abstractPlan.states.end());
    }

    plan.clear();

    if (searchTimeout) {
      solved = search.search(guideStates, plan, searchTimeout);
      if (solved) {
        return true;
      }
    }

    // refine abstraction
    size_t firstUnsolved = search.firstGuideState;
    if (contraction) {
      firstUnsolved = originalIndex[firstUnsolved];
    }
    for (int s = firstUnsolved; s < stepFixed.size(); ++s) {
      if (!stepFixed[s]) {
        abstraction.refine(abstractPlan.steps[s]);
      }
    }
    return false;
  }

  void setSparsification(bool sparsification) {
    this->sparsification = sparsification;
  }

  void setContraction(bool contraction) { this->contraction = contraction; }

  void setAbstractionTimeout(double abstractionTimeout) {
    if (abstractionTimeout < 0) {
      abstractionTimeout = std::numeric_limits<double>::infinity();
    }
    this->abstractionTimeout = abstractionTimeout;
  }

  void setSearchTimeout(double searchTimeout) {
    if (searchTimeout < 0) {
      searchTimeout = std::numeric_limits<double>::infinity();
    }
    this->searchTimeout = searchTimeout;
  }
};
