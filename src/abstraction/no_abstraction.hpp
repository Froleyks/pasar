#pragma once

#include "src/abstraction/abstraction.hpp"

class NoAbstraction {
private:
public:
  size_t numLearnedActions = 0;
  size_t sumSkipped        = 0;
  size_t numRefinements    = 0;
  size_t sumRefineLength   = 0;
  size_t numRefineSteps    = 0;

  NoAbstraction(Problem problem __attribute__((unused))) {}
  static const bool isSatBased = false;

  inline bool
  solve(std::vector<AbstractPlan::Step> &steps __attribute__((unused)),
        double timeLimit
        __attribute__((unused)) = std::numeric_limits<double>::infinity()) {
    return false;
  }

  inline bool fixStep(State &from __attribute__((unused)),
                      AbstractPlan::Step &actions __attribute__((unused)),
                      State &to __attribute__((unused)),
                      std::vector<action_t> &planForStep
                      __attribute__((unused))) {
    return false;
  }

  inline void refine(AbstractPlan::Step &step __attribute__((unused))) {
    return;
  }
};
