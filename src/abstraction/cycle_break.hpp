#pragma once

#include <functional>
#include <unordered_map>

#include "src/abstraction/unrelaxed.hpp"

class CycleBreak : public Unrelaxed {
public:
  CycleBreak(Problem &problem) : Unrelaxed(problem) {
      toggleFrame();
  }

  inline void refine(const AbstractPlan::Step &actions) {
    std::vector<std::pair<action_t, action_t>> edgeList;
    getInterferenceGraph(actions, edgeList);
    refine(edgeList, actions.size());
  }

  inline void refine() {
    addAllInterferances();
  }

  inline void refine(std::vector<std::pair<action_t, action_t>> &edgeList,
                     const size_t numActions) {
    std::vector<unsigned> edges;
    std::vector<unsigned> first;
    std::unordered_map<action_t, unsigned> actionToPacked;
    std::vector<action_t> packedToAction;
    translateToAdjacencyArray(edgeList, numActions, edges, first,
                              actionToPacked, packedToAction);

    std::vector<unsigned> departure;
    std::vector<char> visited(first.size() - 1, false);
    departure.resize(first.size() - 1, 0);
    unsigned time = 0;
    for (unsigned node = 0; node < first.size() - 1; ++node) {
      if (!visited[node]) {
        dfs(edges, first, visited, departure, node, time);
      }
    }

    std::vector<std::pair<action_t, action_t>> mutexes;
    for (size_t node = 0; node < first.size() - 1; ++node) {
      for (size_t edge = first[node]; edge < first[node + 1]; ++edge) {
        if (departure[node] < departure[edges[edge]]) {
          mutexes.emplace_back(packedToAction[node],
                               packedToAction[edges[edge]]);
        }
      }
    }
    addMutexes(mutexes);
  }
};
