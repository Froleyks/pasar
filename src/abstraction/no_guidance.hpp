#pragma once

#include "src/abstraction/abstraction.hpp"

class NoGuidance : public BaseAbstraction {
private:
public:
  static const bool isSatBased = false;

  using BaseAbstraction::BaseAbstraction;

  inline bool
  solve(std::vector<AbstractPlan::Step> &steps,
        double timeLimit = std::numeric_limits<double>::infinity()) {
    return false;
  }

  inline bool fixStep(State &from, AbstractPlan::Step &actions, State &to,
                      std::vector<action_t> &planForStep) {
    return false;
  }

  inline void refine(AbstractPlan::Step &step) { return; }
};
