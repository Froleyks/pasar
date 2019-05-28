#pragma once

#include <functional>
#include <unordered_map>
#include <unordered_set>

#include "src/abstraction/sat_based_abstraction.hpp"

class TrueExist : public SatBasedAbstraction {
private:
  void dfs(const std::vector<unsigned> &edges,
           const std::vector<unsigned> &first,
           std::vector<char> &closedAfterSerach,
           std::vector<unsigned> &departure, const unsigned node,
           unsigned &time) {
    closedAfterSerach[node] = true;
    for (size_t e = first[node]; e < first[node + 1]; ++e) {
      if (!closedAfterSerach[edges[e]]) {
        dfs(edges, first, closedAfterSerach, departure, edges[e], time);
      }
    }
    departure[node] = time++;
  }

  inline bool isDag(const std::vector<unsigned> &edges,
                    const std::vector<unsigned> &first,
                    std::vector<unsigned> &departure) {
    std::vector<char> closedAfterSerach(first.size() - 1, false);
    departure.resize(first.size() - 1, 0);
    unsigned time = 0;
    for (unsigned node = 0; node < first.size() - 1; ++node) {
      if (!closedAfterSerach[node]) {
        dfs(edges, first, closedAfterSerach, departure, node, time);
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

    log(6) << "fixed step, ordering " << actions.size();
    return true;
  }

  class CycleDetection {
    // [0..visiting - 1] are closed
    static constexpr unsigned char closed  = 0;
    static constexpr unsigned char inStack = 1;
    static constexpr unsigned char open =
        std::numeric_limits<unsigned char>::max();

    std::vector<unsigned char> state;

    const std::vector<unsigned> &edges;
    const std::vector<unsigned> &first;

    std::vector<std::vector<unsigned>> mutexes;

    void findMinimalSubCycle(std::vector<unsigned> &cycle) {
      // size is known and not too big, vec bool or even counter invalidation
      // bool might be better
      std::unordered_map<unsigned, size_t> cycleIndex(cycle.size());
      for (size_t c = 0; c < cycle.size(); ++c) {
        cycleIndex[cycle[c]] = c;
      }
      for (size_t i = 0; i < cycle.size(); ++i) {
        unsigned nextInCycle = (i < cycle.size() - 1) ? cycle[i + 1] : cycle[0];
        for (size_t e = first[cycle[i]]; e < first[cycle[i] + 1]; ++e) {
          if (edges[e] == nextInCycle) {
            continue;
          }
          const auto shortcut = cycleIndex.find(edges[e]);
          if (shortcut == cycleIndex.end()) {
            // node not in current cycle
            continue;
          }
          size_t j = shortcut->second;
          assert(cycle[j] == edges[e]);
          // map to index
          if (i < j) {
            // forward edge
            auto begin = cycle.begin() + static_cast<long>(i) + 1;
            auto end   = cycle.begin() + static_cast<long>(j);
            cycle.erase(begin, end);
            i--;
          } else {
            // backward edge
            auto begin = cycle.begin() + static_cast<long>(j);
            auto end   = cycle.begin() + static_cast<long>(i) + 1;
            cycle      = std::vector<unsigned>(begin, end);
            i          = 0;
          }
          cycleIndex.clear();
          cycleIndex.reserve(cycle.size());
          for (size_t c = 0; c < cycle.size(); ++c) {
            cycleIndex[cycle[c]] = c;
          }
          break;
        }
      }
    }

    void searchCycles(unsigned startNode, unsigned char runNumber) {
      const unsigned char closedAfterSerach = runNumber;
      std::vector<std::tuple<unsigned, size_t, bool>> stack;
      assert(stack.empty());
      std::vector<unsigned> mutexSize;
      // node ↦ mutexes it is part of
      std::unordered_map<unsigned, std::vector<size_t>> partOfMutexes;

      state[startNode] = inStack;
      stack.emplace_back(startNode, 0, true);
      size_t e = first[startNode];
      while (!stack.empty()) {
        for (; e < first[std::get<0>(stack.back()) + 1]; ++e) {
          // state[edge[e]] < closedAfterSerach ⇒ closed
          if (state[edges[e]] >= closedAfterSerach) { // ⇒ open
            bool allowed = true;
            if (const auto mutexList = partOfMutexes.find(edges[e]);
                mutexList != partOfMutexes.end()) {
              for (size_t m = 0; m < mutexList->second.size(); ++m) {
                if (!--mutexSize[mutexList->second[m]]) {
                  allowed = false;
                  for (size_t i = 0; i <= m; ++i) {
                    ++mutexSize[mutexList->second[i]];
                  }
                  break;
                }
              }
            }
            if (!allowed) {
              std::get<2>(stack.back()) = false;
              continue;
            }
            state[edges[e]] = inStack;
            stack.emplace_back(edges[e], e, true);
            e = first[edges[e]];
            break;
          } else if (state[edges[e]] == inStack) {
            // found cycle
            size_t cycleBegin = stack.size();
            while (std::get<0>(stack[--cycleBegin]) != edges[e]) {}
            assert(std::get<0>(stack[cycleBegin]) == edges[e]);
            std::vector<unsigned> cycle;
            cycle.reserve(stack.size() - cycleBegin);
            for (size_t i = cycleBegin; i < stack.size(); ++i) {
              cycle.push_back(std::get<0>(stack[i]));
            }
            findMinimalSubCycle(cycle);
            // avoid mutex during current search
            // we are in the cycle we will backjump over it and set the size
            // mutexSize.push_back(cycle.size());
            mutexSize.push_back(0);
            for (auto u : cycle) {
              auto it = partOfMutexes.find(u);
              if (it == partOfMutexes.end()) {
                partOfMutexes[u] = {static_cast<unsigned>(mutexSize.size()) -
                                    1};
              } else {
                it->second.push_back(mutexSize.size() - 1);
              }
            }

            mutexes.push_back(cycle);
            // backjump
            while (std::get<0>(stack.back()) != cycle.back()) {
              state[std::get<0>(stack.back())] = open;

              if (const auto mutexList =
                      partOfMutexes.find(std::get<0>(stack.back()));
                  mutexList != partOfMutexes.end()) {
                for (auto m : mutexList->second) {
                  ++mutexSize[m];
                }
              }
              stack.pop_back();
            }
            state[std::get<0>(stack.back())] = open;
            if (const auto mutexList =
                    partOfMutexes.find(std::get<0>(stack.back()));
                mutexList != partOfMutexes.end()) {
              for (auto m : mutexList->second) {
                ++mutexSize[m];
              }
            }
            e = std::get<1>(stack.back()) + 1;
            stack.pop_back();
            std::get<2>(stack.back()) = false;
            // assertion: the search does not violate any mutex
            break;
          }
        }
        if (e >= first[std::get<0>(stack.back()) + 1]) {
          // backtrack
          // the child next to the current v
          bool closeOnBacktrack = std::get<2>(stack.back());
          if (closeOnBacktrack) {
            state[std::get<0>(stack.back())] = closed;
          } else {
            state[std::get<0>(stack.back())] = closedAfterSerach;
          }
          if (const auto mutexList =
                  partOfMutexes.find(std::get<0>(stack.back()));
              mutexList != partOfMutexes.end()) {
            for (auto m : mutexList->second) {
              ++mutexSize[m];
            }
          }
          e = std::get<1>(stack.back()) + 1;
          stack.pop_back();
          if (stack.empty()) {
            return;
          }
          // transition of closeOnBacktrack property to new back
          std::get<2>(stack.back()) =
              std::get<2>(stack.back()) && closeOnBacktrack;
        }
      }
    }

  public:
    CycleDetection(const std::vector<unsigned> &edges_,
                   const std::vector<unsigned> &first_)
        : edges(edges_), first(first_) {
      state.resize(first.size() - 1, open);
    }

    void allMinimalCycles(std::vector<std::vector<unsigned>> &inMutexes) {
      for (size_t u = 0; u < first.size() - 1; ++u) {
        for (size_t e = first[u]; e < first[u + 1]; ++e) {
          // std::cout << u << " " << edges[e] << std::endl;
        }
      }
      unsigned char currentRunNumber = 2;
      for (unsigned v = 0; v < state.size(); ++v) {
        if (state[v] < currentRunNumber) {
          continue;
        }
        searchCycles(v, currentRunNumber++);
        if (currentRunNumber >= open - 5) {
          for (size_t i = 0; i < state.size(); ++i) {
            state[i] = (state[i] == open) ? open : closed;
          }
          currentRunNumber = 2;
        }
      }
      inMutexes = mutexes;
    }
  };

public:
  TrueExist(Problem &problem, bool cegar = false)
      : SatBasedAbstraction(problem, true) {
    computeValueSupport();
    atMostOneValue();
    initial();
    assumeGoal();
    preconditions();
    effects();

    firstActionToAdd = static_cast<action_t>(problem.numActions);

    if (cegar) {
      toggleFrame();
    } else {
      frame();
      refine({}); // all
    }
  }

  inline bool
  solve(std::vector<AbstractPlan::Step> &steps,
        double timeLimit = std::numeric_limits<double>::infinity()) {
    log(4) << "start solving abstraction";

    double endTime = Logger::getTime() + timeLimit;
    addNewActions();
    size_t makespan = f.getMakespan();
    if (makespan == 1 && initialMakespan_ > 1) {
      makespan = f.increaseMakespan(initialMakespan_ - 1);
    }
    timeLimit   = std::min(endTime - Logger::getTime(), timeLimitPerMakespan_);
    bool solved = f.solve(timeLimit);
    if (solved) {
      log(4) << "solved abstraction in initial makespan " << makespan;
    }
    while (!solved) {
      unsigned increase = std::max(
          static_cast<unsigned>(
              (makespanIncrease_ - 1) * static_cast<double>(makespan) + 0.1),
          1u);
      makespan = f.increaseMakespan(increase);

      timeLimit = std::min(endTime - Logger::getTime(), timeLimitPerMakespan_);
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

  inline bool fixStep(State &from __attribute__((unused)),
                      AbstractPlan::Step &actions,
                      State &to __attribute__((unused)),
                      std::vector<action_t> &planForStep) {
    if (actions.size() < 1) {
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

    bool fixed = tryOrderActions(actions, edgeList, planForStep);
    return fixed;
  }

  // TODO inherit form unrelaxed sat encoding
  inline void refine(const AbstractPlan::Step &actions) {
    std::vector<std::pair<action_t, action_t>> edgeList;
    if (actions.empty()) {
      getInterferenceGraph(edgeList);
    } else {
      getInterferenceGraph(actions, edgeList);
    }
    std::vector<unsigned> edges;
    std::vector<unsigned> first;
    std::unordered_map<action_t, unsigned> actionToPacked(problem_.numActions);
    std::vector<action_t> packedToAction;
    packedToAction.reserve(problem_.numActions);
    translateToAdjacencyArray(edgeList, problem_.numActions, edges, first,
                              actionToPacked, packedToAction);
    std::vector<std::vector<unsigned>> mutexes;
    CycleDetection cycleDetection(edges, first);
    cycleDetection.allMinimalCycles(mutexes);
    std::vector<std::vector<unsigned>> actionMutexes;
    for (auto &m : mutexes) {
      actionMutexes.emplace_back();
      actionMutexes.back().reserve(m.size());
      for (auto a : m) {
        actionMutexes.back().push_back(packedToAction[a]);
      }
    }
    log(3) << "computed all cycles";
    addMutexes(actionMutexes);
  }
};
