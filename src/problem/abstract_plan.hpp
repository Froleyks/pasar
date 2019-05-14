#pragma once

#include <vector>

#include "src/type_defs.hpp"

struct AbstractPlan {
  // one more state than steps
  std::vector<State> states;

  using Step = std::vector<action_t>;
  std::vector<Step> steps;

  void clear() {
    states.clear();
    steps.clear();
  }
};
