#pragma once

#include "src/learn_actions.hpp"
#include "src/problem/problem.hpp"
#include "src/search/search.hpp"
#include "src/type_defs.hpp"
#include "tools/parameter_processor.hpp"

#include "src/search/depth_first_search.hpp"

#include "abstraction/foreach.hpp"

class Pasar {
private:
  // parameters
  int sparsification_ = 1;
  bool contraction_   = false;

  Problem &problem_;

  void abstractActionElimination(std::vector<AbstractPlan::Step> &steps) {
    State state = problem_.initialState;
    for (size_t i = 0; i < steps.size(); ++i) {
      State endPlanState = state;
      std::vector<AbstractPlan::Step> endPlan;
      for (size_t j = i + 1; j < steps.size(); ++j) {
        bool parallelApplicable = true;
        for (size_t a = 0; a < steps[j].size(); ++a) {
          if (!BaseSearch::assignmentHolds(problem_.pre[steps[j][a]],
                                           endPlanState)) {
            parallelApplicable = false;
          }
        }
        if (parallelApplicable) {
          endPlan.push_back(steps[j]);
          for (size_t a = 0; a < steps[j].size(); ++a) {
            BaseSearch::applyAssignment(problem_.eff[steps[j][a]],
                                        endPlanState);
          }
        }
      }
      if (BaseSearch::assignmentHolds(problem_.goal, endPlanState)) {
        steps.erase(steps.begin() + static_cast<long>(i), steps.end());
        steps.insert(steps.end(), endPlan.begin(), endPlan.end());
        i--;
      } else {
        for (size_t a = 0; a < steps[i].size(); ++a) {
          BaseSearch::applyAssignment(problem_.eff[steps[i][a]], state);
        }
      }
    }
  }

  // used to extract states form an abstract plan
  // the extracted states are consistent since the abstract plan was consistent
  // side effect: the step is sparsified
  inline void unapplyActions(AbstractPlan::Step &step, State &state) {
    // vector of bool
    std::vector<unsigned char> keepAction(step.size(), false);
    // remove fulfilled atoms from state
    for (size_t i = 0; i < step.size(); ++i) {
      for (auto [variable, value] : problem_.eff[step[i]]) {
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
      for (auto [variable, value] : problem_.pre[step[i]]) {
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
    State state = problem_.goalState;
    for (size_t s = stepSequence.size() - 1; s != static_cast<size_t>(-1);
         --s) {
      unapplyActions(stepSequence[s], state);
    }

    abstractActionElimination(stepSequence);
  }

  void extractCompleteStates(std::vector<State> &states, Formula &f) {
    states.resize(f.state.size());
    for (size_t t = 0; t < f.state.size(); ++t) {
      states[t].resize(f.state[t].size());
      for (variable_t variable = 0; variable < f.state[t].size(); ++variable) {
        states[t][variable] = f.getStateValue(variable, t);
      }
    }
  }

  void extractPartialStates(std::vector<AbstractPlan::Step> &steps,
                            std::vector<State> &states) {
    states.clear();
    states.reserve(steps.size() + 1);
    State state = problem_.goalState;
    states.insert(states.begin(), state);
    for (size_t s = steps.size() - 1; s != static_cast<size_t>(-1); --s) {
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
    plan.reserve(abstractPlanLength);
    stepFixed.resize(abstractPlanLength, false);
    bool fixedPlan = true;
    for (size_t s = abstractPlanLength - 1; s != static_cast<size_t>(-1); --s) {
      if (abstractPlan.steps[s].empty()) {
        LOG(5) << "step " << s << " empty";
        stepFixed[s] = true;
        continue;
      }
      // the abstract plan might be fixed
      std::vector<action_t> planForStep;
      int fixed =
          abstraction.fixStep(abstractPlan.states[s], abstractPlan.steps[s],
                              abstractPlan.states[s + 1], planForStep);
      if (fixed) {
        LOG(5) << "step " << s << " fixed";
        stepFixed[s] = true;
        plan.insert(plan.begin(), addActionToProblem(problem_, planForStep));
        if (planForStep.size() > 1) {
          abstraction.numLearnedActions++;
          abstraction.sumSkipped += planForStep.size();
        }
      } else {
        LOG(5) << "step " << s << " failed";
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
    for (size_t s = 0; s < planLength; ++s) {
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
        addActionToProblem(problem_, plan, first, s - 1);
        first = planLength;
        // add the last state of every shortcut
        guideStates.push_back(abstractPlan.states[s]);
        originalIndex.push_back(s);
      }
    }
    if (stepFixed.back()) {
      addActionToProblem(problem_, plan, first, planLength - 1);
    }
    guideStates.push_back(abstractPlan.states.back());
    originalIndex.push_back(planLength - 1);
  }

public:
  Pasar(Problem &problem) : problem_(problem) {}

  template <class Abstraction, class Search>
  bool findPlan(Abstraction &abstraction, Search &search,
                std::vector<action_t> &plan, unsigned &sameMakespanCount,
                bool interleave = false) {
    AbstractPlan abstractPlan;
    bool solvedAbstraction = false;
    int abstractionReturn  = false;
    if (interleave) {
      abstractionReturn =
          abstraction.solveAndSearch(abstractPlan.steps, search, plan);
      if (abstractionReturn == 3) {
        return true;
      }
    } else {
      abstractionReturn = abstraction.solve(abstractPlan.steps);
    }
    if (abstractionReturn == 2) {
      sameMakespanCount++;
    } else {
      sameMakespanCount = 0;
    }
    solvedAbstraction = abstractionReturn;

    if (Logger::currentLogLevel() >= 8) {
      std::string levenshtein = "";
      for (auto &s : abstractPlan.steps) {
        for (action_t a : s) {
          levenshtein += std::to_string(a);
          levenshtein += " ";
        }
      }
      LOG(8) << "LEVENSHTEIN: " << levenshtein;
    }

    std::vector<State> guideStates;
    if (!solvedAbstraction) {
      LOG(4) << "faild abstraction ";
      guideStates = {problem_.goalState};
      return search.search(guideStates, plan);
    }

    switch (sparsification_) {
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
      State state = problem_.goalState;
      for (size_t s = abstractPlan.steps.size() - 1;
           s != static_cast<size_t>(-1); --s) {
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
      LOG(3) << "fixed abstract plan";
      return true;
    }

    /// plan contraction
    // used to find the last visited guide state
    std::vector<size_t> originalIndex;
    if (contraction_) {
      LOG(4) << "contraction on plan length " << abstractPlan.steps.size();

      abstractPlanContraction(stepFixed, plan, abstractPlan, guideStates,
                              originalIndex);
    } else {
      // ignore first
      guideStates = std::vector<State>(abstractPlan.states.begin() + 1,
                                       abstractPlan.states.end());
    }

    plan.clear();

    solved = search.search(guideStates, plan);
    if (solved) {
      return true;
    }

    LOG(4) << "search failed";

    // refine abstraction
    size_t firstUnsolved = search.firstGuideState;
    if (contraction_) {
      firstUnsolved = originalIndex[firstUnsolved];
    }
    for (size_t s = firstUnsolved; s < stepFixed.size(); ++s) {
      if (!stepFixed[s]) {
        LOG(4) << "refine step " << s;
        abstraction.refine(abstractPlan.steps[s]);
      }
    }
    return false;
  }

  void setSparsification(int sparsification) {
    sparsification_ = sparsification;
  }

  void setContraction(bool contraction) { contraction_ = contraction; }
};
