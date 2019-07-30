#pragma once

#include <functional>
#include <unordered_map>

#include "src/abstraction/unrelaxed.hpp"

class LeafReduction : public Unrelaxed {
public:
  static inline void
  leafReduction(std::vector<std::pair<action_t, action_t>> &edgeList) {
    if (edgeList.empty()) {
      return;
    }
    std::unordered_map<action_t, action_t> packed;
    action_t numNodes = 0;
    action_t last     = edgeList[0].first;
    packed[last]      = numNodes++;
    for (auto [u, v] : edgeList) {
      if (last != u) {
        last         = u;
        packed[last] = numNodes++;
      }
    }

    std::vector<size_t> outDegree(numNodes, 0);
    std::vector<std::vector<action_t>> inEdges(numNodes);
    for (auto [u, v] : edgeList) {
      if (auto it = packed.find(v); it != packed.end()) {
        outDegree[packed[u]]++;
        inEdges[it->second].push_back(packed[u]);
      }
    }
    std::vector<action_t> leaves;
    leaves.reserve(numNodes);
    for (action_t u = 0; u < outDegree.size(); ++u) {
      if (!outDegree[u]) {
        leaves.push_back(u);
      }
    }
    for (size_t i = 0; i < leaves.size(); ++i) {
      for (auto u : inEdges[leaves[i]]) {
        if (!--outDegree[u]) {
          leaves.push_back(u);
        }
      }
    }
    std::vector<std::pair<action_t, action_t>> cycles;
    for (auto e : edgeList) {
      if (auto it = packed.find(e.second);
          it != packed.end() && outDegree[it->second]) {
        cycles.push_back(e);
      }
    }
    LOG(4) << "added " << cycles.size() << " of " << edgeList.size()
           << " mutexes";
    edgeList = cycles;
  }

  LeafReduction(Problem &problem, bool cegar = false) : Unrelaxed(problem) {
    if (cegar) {
      toggleFrame();
    } else {
      frame();
      refine(); // all
    }
  }

  inline void refine(const AbstractPlan::Step &actions) {
    std::vector<std::pair<action_t, action_t>> edgeList;
    getInterferenceGraph(actions, edgeList);
    refine(edgeList);
  }

  inline void refine() {
    std::vector<std::pair<action_t, action_t>> edgeList;
    getInterferenceGraph(edgeList);
    LOG(3) << "adding all interferances on " << edgeList.size() << " edges";
    refine(edgeList);
  }

  inline void refine(std::vector<std::pair<action_t, action_t>> &edgeList) {
    leafReduction(edgeList);
    addMutexes(edgeList);
  }
};
