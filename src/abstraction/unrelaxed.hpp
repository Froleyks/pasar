#pragma once

#include <functional>
#include <unordered_map>

#include "src/abstraction/sat_based_abstraction.hpp"

class Unrelaxed : public SatBasedAbstraction {
protected:
  void dfs(const std::vector<unsigned> &edges,
           const std::vector<unsigned> &first, std::vector<char> &visited,
           std::vector<unsigned> &departure, const unsigned node,
           unsigned &time) {
    visited[node] = true;
    for (size_t e = first[node]; e < first[node + 1]; ++e) {
      if (!visited[edges[e]]) {
        dfs(edges, first, visited, departure, edges[e], time);
      }
    }
    departure[node] = time++;
  }

  inline bool isDag(const std::vector<unsigned> &edges,
                    const std::vector<unsigned> &first,
                    std::vector<unsigned> &departure) {
    std::vector<char> visited(first.size() - 1, false);
    departure.resize(first.size() - 1, 0);
    unsigned time = 0;
    for (unsigned node = 0; node < first.size() - 1; ++node) {
      if (!visited[node]) {
        dfs(edges, first, visited, departure, node, time);
      }
    }

    for (size_t node = 0; node < first.size() - 1; ++node) {
      for (size_t edge = first[node]; edge < first[node + 1]; ++edge) {
        if (departure[node] < departure[edges[edge]]) {
          return false;
        }
      }
    }
    return true;
  }

  inline bool
  tryOrderActions(const std::vector<unsigned> &actions,
                  std::vector<std::pair<action_t, action_t>> &edgeList,
                  std::vector<action_t> &planForStep) {
    std::vector<unsigned> edges;
    std::vector<unsigned> first;
    std::unordered_map<action_t, action_t> translate(actions.size());
    translateToAdjacencyArray(edgeList, actions.size(), edges, first,
                              translate);

    std::vector<unsigned> ranking;

    bool solved;
    solved = isDag(edges, first, ranking);
    if (!solved) {
      return false;
    }
    size_t numRanked = ranking.size();
    planForStep.reserve(actions.size());
    planForStep.resize(numRanked);
    for (auto a : actions) {
      auto it = translate.find(a);
      if (it == translate.end()) {
        planForStep.push_back(a);
      } else {
        unsigned packedA                              = it->second;
        planForStep[numRanked - 1 - ranking[packedA]] = a;
      }
    }

    LOG(6) << "fixed step, ordering " << actions.size();
    return true;
  }

public:
  Unrelaxed(Problem &problem) : SatBasedAbstraction(problem, true) {
    computeValueSupport();
    atMostOneValue();
    initial();
    assumeGoal();
    preconditions();
    effects();

    firstActionToAdd = static_cast<action_t>(problem.numActions);
  }

  inline bool fixStep(State &from __attribute__((unused)),
                      AbstractPlan::Step &actions,
                      State &to __attribute__((unused)),
                      std::vector<action_t> &planForStep) {
    if (actions.size() < 1) {
      LOG(6) << "fixed step, only action ";
      planForStep = actions;
      return true;
    }

    std::vector<std::pair<action_t, action_t>> edgeList;
    getInterferenceGraph(actions, edgeList);
    if (edgeList.empty()) {
      LOG(6) << "fixed step, no interferance on " << actions.size()
             << " actions";
      planForStep = actions;
      return true;
    }

    return tryOrderActions(actions, edgeList, planForStep);
  }
};
