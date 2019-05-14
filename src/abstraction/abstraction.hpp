#pragma once

#include "formular.hpp"
#include "src/problem/abstract_plan.hpp"
#include "src/problem/problem.hpp"
#include <unordered_set>

class BaseAbstraction {
private:
protected:
  Problem &problem;

  // (variable, value)s ↦ actions; v → a ⇔ v ∈ eff(a)
  std::vector<std::vector<std::vector<action_t>>> valueSupport;

  inline void computeValueSupport() {
    valueSupport.resize(problem.numVariables);
    for (int v = 0; v < problem.numValues.size(); ++v) {
      valueSupport[v].resize(problem.numValues[v]);
    }
    for (int v = problem.numValues.size(); v < problem.numVariables; ++v) {
      valueSupport[v].resize(2);
    }
    for (size_t a = 0; a < problem.eff.size(); ++a) {
      for (auto [variable, value] : problem.eff[a]) {
        valueSupport[variable][value].push_back(a);
      }
    }
  }

  void
  getInterferenceGraph(std::vector<std::pair<action_t, action_t>> &edgeList) {
    edgeList.clear();
    for (action_t a = 0; a < problem.numActions; ++a) {
      std::unordered_set<unsigned> hasEdge(problem.numActions);
      for (auto p : problem.pre[a]) {
        for (unsigned otherValue = 0; otherValue < valueSupport[p.first].size();
             ++otherValue) {
          if (otherValue == p.second)
            continue;
          for (action_t otherA : valueSupport[p.first][otherValue]) {
            // is a different action and has no edge from a jet
            if (a != otherA && hasEdge.find(otherA) == hasEdge.end()) {
              hasEdge.insert(otherA);
              // a has to be executed before otherA
              edgeList.emplace_back(a, otherA);
            }
          }
        }
      }
    }
  }

  void
  getInterferenceGraph(const std::vector<action_t> &actions,
                       std::vector<std::pair<action_t, action_t>> &edgeList) {
    edgeList.clear();
    std::unordered_set<action_t> actionSet(actions.begin(), actions.end());
    for (auto a : actions) {
      std::unordered_set<unsigned> hasEdge(problem.numActions);
      for (auto p : problem.pre[a]) {
        for (unsigned otherValue = 0; otherValue < valueSupport[p.first].size();
             ++otherValue) {
          if (otherValue == p.second)
            continue;
          for (action_t otherA : valueSupport[p.first][otherValue]) {
            // is a different action and has no edge from a jet
            if (a != otherA && actionSet.find(otherA) != actionSet.end() &&
                hasEdge.find(otherA) == hasEdge.end()) {
              hasEdge.insert(otherA);
              // a has to be executed before otherA
              edgeList.emplace_back(a, otherA);
            }
          }
        }
      }
    }
  }


public:
  BaseAbstraction(Problem &problem) : problem(problem) {}
};
