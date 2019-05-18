#pragma once

#include "src/abstraction/abstraction.hpp"

class NoGuidance : public BaseAbstraction {
private:
public:
  static const bool isSatBased = false;

  using BaseAbstraction::BaseAbstraction;

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
