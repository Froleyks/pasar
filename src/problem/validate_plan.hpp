#pragma once

#include <iostream>
#include <vector>

#include "src/problem/problem.hpp"
#include "src/type_defs.hpp"

template <bool print = false>
bool validatePlan(const std::vector<action_t> &plan, const Problem &problem) {
  State state;
  state.resize(problem.numVariables);
  state = problem.initialState;
  for (size_t i = 0; i < plan.size(); ++i) {
    for (auto [index, value] : problem.pre[plan[i]]) {
      if (state[index] != value) {
        std::cout << "invalid " << i << " " << plan[i] << std::endl;
        return false;
      }
    }
    if constexpr (print) {
      std::cout << "plan " << i << " " << plan[i] << std::endl;
    }
    for (auto [index, value] : problem.eff[plan[i]]) {
      state[index] = value;
    }
  }

  // goal satisfied?
  for (size_t i = 0; i < problem.goalState.size(); ++i) {
    if (problem.goalState[i] != unassigned &&
        state[i] != problem.goalState[i]) {
      return false;
    }
  }
  return true;
}
