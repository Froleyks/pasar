#pragma once

#include <functional>
#include <unordered_map>

#include "src/abstraction/sat_based_abstraction.hpp"

class CegarForeach : public SatBasedAbstraction {
private:
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
          first.push_back(edges.size());
        }
        auto packedV = translate.find(v);
        if (packedV == translate.end()) {
          // do nothing. those are leaves of the graph
          continue;
        }
        edges.push_back(packedV->second);
      }
    }
    first.push_back(edges.size());
  }

  void DFSinDAG(const std::vector<unsigned> &edges,
                const std::vector<unsigned> &first, std::vector<char> &visited,
                std::vector<unsigned> &departure, const unsigned node,
                unsigned &time) {
    visited[node] = true;
    for (size_t e = first[node]; e < first[node + 1]; ++e) {
      if (!visited[edges[e]]) {
        DFSinDAG(edges, first, visited, departure, edges[e], time);
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
    for (size_t node = 0; node < first.size() - 1; ++node) {
      if (!visited[node]) {
        DFSinDAG(edges, first, visited, departure, node, time);
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

    std::vector<std::pair<action_t, action_t>> conflictEdges;
    bool solved;
    solved = isDag(edges, first, ranking);
    if (!solved) {
      return false;
    }
    unsigned numRanked = ranking.size();
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

    log(6) << "fixed step, ordering " << actions.size();
    return true;
  }

  inline void addNewActions() {
    f.addVarsForActions(problem.numActions - firstActionToAdd);

    std::vector<std::pair<variable_t, value_t>> updatedSupports;
    updateValueSupport(firstActionToAdd, updatedSupports);

    // remove duplicates
    std::sort(updatedSupports.begin(), updatedSupports.end());
    updatedSupports.erase(
        std::unique(updatedSupports.begin(), updatedSupports.end()),
        updatedSupports.end());

    preconditions(firstActionToAdd);
    effects(firstActionToAdd);

    for (auto [variable, value] : updatedSupports) {
      updateFrame(variable, value);
    }
    firstActionToAdd = problem.numActions;
  }

public:
  CegarForeach(Problem &problem) : SatBasedAbstraction(problem, true) {
    computeValueSupport();
    atMostOneValue();
    initial();
    assumeGoal();
    preconditions();
    toggleFrame();
    effects();

    firstActionToAdd = problem.numActions;
  }

  inline bool
  solve(std::vector<AbstractPlan::Step> &steps,
        double timeLimit = std::numeric_limits<double>::infinity()) {
    log(4) << "start solving abstraction";

    double endTime = Logger::getTime() + timeLimit;
    addNewActions();
    unsigned makespan = f.getMakespan();
    if (makespan == 1 && initialMakespan > 1) {
      makespan = f.increaseMakespan(initialMakespan - 1);
    }
    timeLimit   = std::min(endTime - Logger::getTime(), timeLimitPerMakespan);
    bool solved = f.solve(timeLimit);
    if (solved) {
      log(4) << "solved abstraction in initial makespan " << makespan;
    }
    while (!solved) {
      int increase =
          std::max((int)(makespan * (makespanIncrease - 1) + 0.1), 1);

      makespan = f.increaseMakespan(increase);

      timeLimit = std::min(endTime - Logger::getTime(), timeLimitPerMakespan);
      log(5) << "start solving makespan " << makespan;
      solved = f.solve(timeLimit);
    }
    if (solved) {
      log(4) << "solved abstraction in makespan " << makespan;
      extractStepSequence(steps);
    } else {
      log(4) << "failed in makespan " << makespan;
    }
    return solved;
  }

  inline bool fixStep(State &from, AbstractPlan::Step &actions, State &to,
                      std::vector<action_t> &planForStep) {
    if (actions.empty()) {
      log(6) << "fixed step, empty";
      planForStep = {noop};
      return true;
    }

    if (actions.size() == 1) {
      log(6) << "fixed step, only action ";
      planForStep = actions;
      return true;
    }

    std::vector<std::pair<action_t, action_t>> edgeList;
    getInterferenceGraph(actions, edgeList);
    if (edgeList.empty()) {
      log(6) << "fixed step, no interferance on " << actions.size()
             << " actions";
      planForStep = actions;
      return true;
    }

    return tryOrderActions(actions, edgeList, planForStep);
  }

  inline void refine(AbstractPlan::Step &actions) {
    std::vector<std::pair<action_t, action_t>> edgeList;
    getInterferenceGraph(actions, edgeList);
    addMutexes(f, edgeList);
  }
};
