#pragma once

#include <functional>
#include <unordered_map>

#include "src/abstraction/unrelaxed.hpp"

class FastForward : public SatBasedAbstraction {
private:
  inline void relaxedPreconditions(action_t firstAction = 0) {
    for (action_t a = firstAction; a < f.action[0].size(); ++a) {
      // a_t => pre(a)_t
      for (auto [variable, value] : problem_.pre[a]) {
        // don't assert negative preconditions for predicates
        if (variable >= problem_.numValues.size() && value) {
          // is predicate
          // value == 0 ⇒ for sas predicates the second value is false(ish)
          continue;
        }
        f.addA(a, false);
        f.addS(variable, value);
        f.close();
      }
    }
  }

  inline void relaxedEffects(action_t firstAction = 0) {
    for (action_t a = firstAction; a < f.action[0].size(); ++a) {
      // a_t => eff(a)_t+1
      for (auto [variable, value] : problem_.eff[a]) {
        // don't encode negative effects for variables that only hold one value
        if (variable >= problem_.numValues.size() && value) {
          // is predicate
          // for sas predicates the second value is false(ish)
          continue;
        }
        f.addA(a, false);
        f.addS(variable, value, true, true);
        f.close();
      }
    }
  }

public:
  FastForward(Problem &problem) : SatBasedAbstraction(problem) {
    computeValueSupport();
    initial();
    assumeGoal();
    relaxedPreconditions();
    relaxedEffects();
    frame();

    firstActionToAdd = static_cast<action_t>(problem.numActions);
  }

  inline bool fixStep(State &from __attribute__((unused)),
                      AbstractPlan::Step &actions __attribute__((unused)),
                      State &to __attribute__((unused)),
                      std::vector<action_t> &planForStep
                      __attribute__((unused))) {
    return false;
  }

  inline void refine(const AbstractPlan::Step &actions
                     __attribute__((unused))) {
    return;
  }
};
