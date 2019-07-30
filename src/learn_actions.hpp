#pragma once

#include <vector>

#include "problem/problem.hpp"
#include "type_defs.hpp"

inline void contractPlan(Problem &problem, const std::vector<action_t> &plan,
                         size_t first, size_t last, Assignment &pre,
                         Assignment &eff) {
  State expPre(problem.numVariables, unassigned);
  State expEff(problem.numVariables, unassigned);
  for (size_t s = first; s <= last; ++s) {
    for (auto [variable, value] : problem.pre[plan[s]]) {
      if (expEff[variable] != value) {
        expPre[variable] = value;
      }
    }
    for (auto [variable, value] : problem.eff[plan[s]]) {
      expEff[variable] = value;
    }
  }
  for (size_t i = 0; i < expPre.size(); ++i) {
    if (expPre[i] != unassigned) {
      pre.emplace_back(i, expPre[i]);
    }
  }
  for (size_t i = 0; i < expEff.size(); ++i) {
    if (expEff[i] != unassigned) {
      eff.emplace_back(i, expEff[i]);
    }
  }
}

inline action_t addActionToProblem(Problem &problem,
                                   const std::vector<action_t> &plan,
                                   size_t first, size_t last) {
  if (last == first) {
    // only one action in plan
    return plan[last];
  }
  problem.numActions++;
  problem.pre.emplace_back();
  problem.eff.emplace_back();
  contractPlan(problem, plan, first, last, problem.pre.back(),
               problem.eff.back());

  // add witness
  problem.witness.push_back(
      std::vector<action_t>(plan.begin() + static_cast<long>(first),
                            plan.begin() + static_cast<long>(last) + 1));

  LOG(5) << "learned action " << problem.pre.size() - 1 << " skips "
         << plan.size();
  return static_cast<action_t>(problem.pre.size() - 1);
}

inline action_t addActionToProblem(Problem &problem,
                                   const std::vector<action_t> &plan) {
  return addActionToProblem(problem, plan, 0, plan.size() - 1);
}
