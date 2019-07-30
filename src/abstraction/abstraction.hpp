#pragma once

#include "formular.hpp"
#include "src/problem/abstract_plan.hpp"
#include "src/problem/problem.hpp"

#include <unordered_set>

class BaseAbstraction {
private:
protected:
  Problem &problem_;

  // (variable, value)s ↦ actions; v → a ⇔ v ∈ eff(a)
  std::vector<std::vector<std::vector<action_t>>> valueSupport;

  inline void computeValueSupport() {
    valueSupport.resize(problem_.numVariables);
    for (size_t v = 0; v < problem_.numValues.size(); ++v) {
      valueSupport[v].resize(problem_.numValues[v]);
    }
    for (size_t v = problem_.numValues.size(); v < problem_.numVariables; ++v) {
      valueSupport[v].resize(2);
    }
    for (action_t a = 0; a < problem_.eff.size(); ++a) {
      for (auto [variable, value] : problem_.eff[a]) {
        valueSupport[variable][value].push_back(a);
      }
    }
  }

  inline void updateValueSupport(
      action_t firstAciton,
      std::vector<std::pair<variable_t, value_t>> &updatedSupports) {
    for (action_t a = firstAciton; a < problem_.eff.size(); ++a) {
      for (auto [variable, value] : problem_.eff[a]) {
        updatedSupports.emplace_back(variable, value);
        valueSupport[variable][value].push_back(a);
      }
    }
  }

  // leaves are not translated to nodes
  inline void translateToAdjacencyArray(
      const std::vector<std::pair<action_t, action_t>> &edgeList,
      const size_t numActions, std::vector<unsigned> &edges,
      std::vector<unsigned> &first,
      std::unordered_map<action_t, action_t> &translate) {
    edges.reserve(edgeList.size());
    first.reserve(numActions + 1);
    // assertion: edgeList are ordered by first packedIndex
    unsigned packedIndex         = 0;
    unsigned last                = edgeList[0].first;
    translate[edgeList[0].first] = packedIndex++;
    // first[0]                  = 0;
    for (size_t i = 1; i < edgeList.size(); ++i) {
      if (last != edgeList[i].first) {
        last = edgeList[i].first;
        // first[packed_index]       = i;
        translate[edgeList[i].first] = packedIndex++;
      }
    }
    last = 0;
    first.push_back(0);
    {
      for (auto [u, v] : edgeList) {
        action_t packedU = translate[u];
        if (last != packedU) {
          last = packedU;
          first.push_back(static_cast<unsigned>(edges.size()));
        }
        auto packedV = translate.find(v);
        if (packedV == translate.end()) {
          // do nothing. those are leaves of the graph
          continue;
        }
        edges.push_back(packedV->second);
      }
    }
    first.push_back(static_cast<unsigned>(edges.size()));
  }

  // leaves are not translated to nodes
  inline void translateToAdjacencyArray(
      const std::vector<std::pair<action_t, action_t>> &edgeList,
      const size_t numActions, std::vector<unsigned> &edges,
      std::vector<unsigned> &first,
      std::unordered_map<action_t, action_t> &translate,
      std::vector<action_t> &translateInveres) {
    translateInveres.reserve(numActions);
    edges.reserve(edgeList.size());
    first.reserve(numActions + 1);
    // assertion: edgeList are ordered by first packedIndex
    unsigned packedIndex = 0;
    unsigned last        = edgeList[0].first;
    translateInveres.push_back(edgeList[0].first);
    translate[edgeList[0].first] = packedIndex++;
    // first[0]                  = 0;
    for (size_t i = 1; i < edgeList.size(); ++i) {
      if (last != edgeList[i].first) {
        last = edgeList[i].first;
        // first[packed_index]       = i;
        translateInveres.push_back(edgeList[i].first);
        translate[edgeList[i].first] = packedIndex++;
      }
    }
    last = 0;
    first.push_back(0);
    {
      for (auto [u, v] : edgeList) {
        action_t packedU = translate[u];
        if (last != packedU) {
          last = packedU;
          first.push_back(static_cast<unsigned>(edges.size()));
        }
        auto packedV = translate.find(v);
        if (packedV == translate.end()) {
          // do nothing. those are leaves of the graph
          continue;
        }
        edges.push_back(packedV->second);
      }
    }
    first.push_back(static_cast<unsigned>(edges.size()));
  }

  void
  getInterferenceGraph(std::vector<std::pair<action_t, action_t>> &edgeList) {
    edgeList.clear();
    for (action_t a = 0; a < problem_.numActions; ++a) {
      std::unordered_set<unsigned> hasEdge(problem_.numActions);
      for (auto p : problem_.pre[a]) {
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
      std::unordered_set<unsigned> hasEdge(actions.size());
      for (auto p : problem_.pre[a]) {
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
  // logging
  size_t numLearnedActions = 0;
  size_t sumSkipped        = 0;
  size_t numRefinements    = 0;
  size_t sumRefineLength   = 0;
  size_t numRefineSteps    = 0;

  BaseAbstraction(Problem &problem) : problem_(problem) {}
  ~BaseAbstraction() {
    LOG(3) << "Abstraction learned " << numLearnedActions << " actions";
    if (sumSkipped) {
      LOG(3) << "average length "
             << static_cast<double>(sumSkipped) /
                    static_cast<double>(numLearnedActions);
    }
    LOG(3) << "Added " << numRefinements << " interferances in "
           << numRefineSteps << " steps";
    LOG(3) << "average length "
           << static_cast<double>(sumRefineLength) /
                  static_cast<double>(numRefinements);
  }
};
