#pragma once

#include <functional>
#include <unordered_map>
#include <unordered_set>

#include "src/abstraction/unrelaxed.hpp"
#include "src/abstraction/leaf_reduction.hpp"


class TrueExist : public Unrelaxed {
private:
  // TODO The correctness of this algorithm is not proven!
  class CycleDetection {
    //     Minimal Cycles
    // Given a directed graph G = (V,E) without self-loops, find all minimal
    // (inclusion-wise) subsets of V for which the induced subgraph of G
    // contains a cycle. Those cycles are always chord free in G.

    // The algorithm consist of three steps
    // 1) shortCycleElimination
    //    All cycles of length two are identified by sorting the give edge list.
    //    The cycles are removed from the graph and the connected nodes are
    //    inserted in each others blockList.
    // 2) leafReduction / SCC TODO
    // 3) minimal cycle detection with exponential DFS (revisiting nodes) using
    // the blockLists introduced in step 1.

    // [0.. current visited - 1] are closed
    enum State : unsigned char {
      closed,
      firstVisited,
      // used to close nodes that have been visited without iterating over them
      // again visited (blockedOpen, open)
      inStack = std::numeric_limits<unsigned char>::max() - 1,
      open    = std::numeric_limits<unsigned char>::max()
    };

    // Each node in V has a state
    std::vector<State> state;

    // adjacency array
    const std::vector<unsigned> &edges;
    const std::vector<unsigned> &first;

    // one for each node
    // if a node is inserted into the stack, nodes in its block list may not be
    // visited
    const std::vector<std::vector<unsigned>> &blockList;

    bool findMinimalSubCycle(std::vector<unsigned> &cycle) {
      // map to index in cycle
      std::unordered_map<unsigned, size_t> cycleIndex(cycle.size());
      for (size_t c = 0; c < cycle.size(); ++c) {
        cycleIndex[cycle[c]] = c;
      }
      for (auto v : cycle) {
        for (auto u : blockList[v]) {
          if (cycleIndex.find(u) != cycleIndex.end()) {
            return false;
          }
        }
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
      return true;
    }

    inline void unblock(
        const std::unordered_map<unsigned, std::vector<size_t>> &partOfMutexes,
        const unsigned backtrackedNode, std::vector<int> &mutexSize) {
      if (const auto mutexList = partOfMutexes.find(backtrackedNode);
          mutexList != partOfMutexes.end()) {
        for (auto m : mutexList->second) {
          ++mutexSize[m];
        }
      }
    }

    void searchCycles(unsigned startNode, const State visited,
                      std::vector<std::vector<unsigned>> &mutexes) {
      // (node, edgeIndex, deleteOnBacktrack)
      // edgeIndex: The index of the edge the node was visited by
      // deleteOnBacktrack: set when we backtrack because we have no child or
      // all children have deleteOnBacktrack set, used to eliminate DAG
      // structures on first visit
      std::vector<std::tuple<unsigned, unsigned, bool>> stack;
      // the size of each currently relevant mutex, decremented if a node in the
      // mutex is inserted into the stack
      std::vector<int> mutexSize;
      // node ↦ mutexes it is part of
      std::unordered_map<unsigned, std::vector<size_t>> partOfMutexes;

      state[startNode] = inStack;
      stack.emplace_back(startNode, 0, true);
      unsigned e = first[startNode];
      while (!stack.empty()) {
        for (; e < first[std::get<0>(stack.back()) + 1]; ++e) {
          if (state[edges[e]] < visited) { // ⇒ closed
            continue;
          }
          if (state[edges[e]] == inStack) {
            // found cycle
            size_t cycleBegin = stack.size();
            while (std::get<0>(stack[--cycleBegin]) != edges[e]) {}
            bool foundNewCycle = true;
            if (const auto mutexList = partOfMutexes.find(edges[e]);
                mutexList != partOfMutexes.end()) {
              for (size_t m = 0; m < mutexList->second.size(); ++m) {
                if (mutexSize[mutexList->second[m]] == 1) {
                  foundNewCycle = false;
                  break;
                }
                if (mutexSize[mutexList->second[m]] <= 0) {
                  bool fixedThisViolation = false;
                  for (size_t nonCycle = cycleBegin - 1;
                       nonCycle != static_cast<size_t>(-1); --nonCycle) {
                    if (fixedThisViolation) {
                      break;
                    }
                    const auto nonCycleMutexes =
                        partOfMutexes.find(std::get<0>(stack[nonCycle]));
                    if (nonCycleMutexes == partOfMutexes.end()) {
                      continue;
                    }
                    for (auto nonCycleM : nonCycleMutexes->second) {
                      if (mutexList->second[m] == nonCycleM) {
                        fixedThisViolation = true;
                        break;
                      }
                    }
                  }
                  if (!fixedThisViolation) {
                    foundNewCycle = false;
                    break;
                  }
                }
              }
            }
            if (!foundNewCycle) {
              continue;
            }
            std::vector<unsigned> cycle;
            cycle.reserve(stack.size() - cycleBegin);
            for (size_t i = cycleBegin; i < stack.size(); ++i) {
              cycle.push_back(std::get<0>(stack[i]));
            }
            foundNewCycle = findMinimalSubCycle(cycle);
            if (!foundNewCycle) {
              continue;
            }
            mutexes.push_back(cycle);
            // avoid mutex during current search
            // it will stay violated until the search backtracks over its
            // first node the correct size will be set by backtracking over
            // the nodes
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
          } else {
            // is a mutex violated by visiting this node? also update number
            // of nodes allowed in mutex
            bool allowed = true;
            if (const auto mutexList = partOfMutexes.find(edges[e]);
                mutexList != partOfMutexes.end()) {
              for (size_t m = 0; m < mutexList->second.size(); ++m) {
                if (--mutexSize[mutexList->second[m]] <= 0) {
                  allowed = false;
                  for (size_t i = 0; i <= m; ++i) {
                    ++mutexSize[mutexList->second[i]];
                  }
                  break;
                }
              }
            }
            if (!allowed) {
              // the node and its parents may not be deleted on backtrack
              // since they may be part of other cycles
              std::get<2>(stack.back()) = false;
              continue;
            }
            // open
            state[edges[e]] = inStack;
            stack.emplace_back(edges[e], e, true);
            e = first[edges[e]];
            break;
          }
        }
        if (e >= first[std::get<0>(stack.back()) + 1]) {
          // backtrack
          bool closeOnBacktrack = std::get<2>(stack.back());
          if (closeOnBacktrack) {
            state[std::get<0>(stack.back())] = closed;
          } else {
            // visited nodes will only be closed after this search is
            // completed
            state[std::get<0>(stack.back())] = visited;
          }
          unblock(partOfMutexes, std::get<0>(stack.back()), mutexSize);
          // the next child
          e = std::get<1>(stack.back()) + 1;
          stack.pop_back();
          if (stack.empty()) {
            return;
          }
          // propagation of closeOnBacktrack to parent
          std::get<2>(stack.back()) =
              std::get<2>(stack.back()) && closeOnBacktrack;
        }
      }
    }

  public:
    CycleDetection(const std::vector<unsigned> &edges_,
                   const std::vector<unsigned> &first_,
                   const std::vector<std::vector<unsigned>> &blockList_)
        : edges(edges_), first(first_), blockList(blockList_) {
      LOG(6) << "all minimal cycles on " << edges_.size() << " edges";
      state.resize(first.size() - 1, open);
    }

    void allMinimalCycles(std::vector<std::vector<unsigned>> &mutexes) {
      unsigned char currentRunNumber = firstVisited;
      for (unsigned v = 0; v < state.size(); ++v) {
        if (state[v] < currentRunNumber) {
          continue;
        }
        searchCycles(v, static_cast<State>(currentRunNumber++), mutexes);
        if (currentRunNumber == open) {
          for (size_t i = 0; i < state.size(); ++i) {
            state[i] = (state[i] == open) ? open : closed;
          }
          currentRunNumber = firstVisited;
        }
      }
    }
  };

public:
  TrueExist(Problem &problem, bool cegar = false) : Unrelaxed(problem) {
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
    refine(edgeList, actions.size());
  }

  inline void refine() {
    std::vector<std::pair<action_t, action_t>> edgeList;
    getInterferenceGraph(edgeList);
    LOG(3) << "adding all interferances on " << edgeList.size() << " edges";
    refine(edgeList, problem_.numActions);
  }

  inline void
  shortCycleElimination(std::vector<std::pair<action_t, action_t>> &edgeList,
                        std::vector<std::vector<unsigned>> &actionMutexes) {
    std::sort(edgeList.begin(), edgeList.end(), [](auto a, auto b) {
      action_t minA = std::min(a.first, a.second);
      action_t maxA = std::max(a.first, a.second);
      action_t minB = std::min(b.first, b.second);
      action_t maxB = std::max(b.first, b.second);
      return minA < minB ||
             (minA == minB &&
              (maxA < maxB || (maxA == maxB && a.first < b.first)));
    });
    size_t i;
    size_t insert = 0;
    for (i = 0; i < edgeList.size() - 1; ++i) {
      if (edgeList[i].first == edgeList[i + 1].second &&
          edgeList[i].second == edgeList[i + 1].first) {
        actionMutexes.push_back({edgeList[i].first, edgeList[i].second});
        i++;
      } else {
        edgeList[insert++] = edgeList[i];
      }
    }
    if (i < edgeList.size()) {
      edgeList[insert++] = edgeList[i];
    }
    edgeList.erase(edgeList.begin() + static_cast<long>(insert),
                   edgeList.end());
    edgeList.shrink_to_fit();

    LOG(6) << "added " << actionMutexes.size() << " trivial mutexes";
  }

  bool isSubset(const std::vector<action_t> &smaller,
                const std::vector<action_t> &bigger) {
    size_t i = 0;
    size_t j = 0;
    for (i = 0; i < smaller.size(); i++) {
      for (j = 0; j < bigger.size(); j++) {
        if (smaller[i] == bigger[j])
          break;
      }
      if (j == bigger.size())
        return 0;
    }
    return 1;
  }

  inline void refine(std::vector<std::pair<action_t, action_t>> &edgeList,
                     const size_t numActions) {
    if (edgeList.empty()) {
      return;
    }

    std::vector<std::vector<unsigned>> actionMutexes;
    shortCycleElimination(edgeList, actionMutexes);

    if (edgeList.empty()) {
      addMutexes(actionMutexes);
      return;
    }

    const size_t numNonShort = edgeList.size();
    LeafReduction::leafReduction(edgeList);
    LOG(4) << "leaf reduction removed " << numNonShort - edgeList.size();

    // build adjacency array from edge list with non consecutive indices
    std::vector<unsigned> edges;
    std::vector<unsigned> first;
    std::unordered_map<action_t, unsigned> actionToPacked(numActions);
    std::vector<action_t> packedToAction;
    // necessary for translation
    std::sort(edgeList.begin(), edgeList.end());
    translateToAdjacencyArray(edgeList, numActions, edges, first,
                              actionToPacked, packedToAction);
    // mutexes found during shortCycleElimination must be considered during
    // cycle search
    std::vector<std::vector<unsigned>> blockList(packedToAction.size());
    for (auto m : actionMutexes) {
      auto u = actionToPacked.find(m[0]);
      auto v = actionToPacked.find(m[1]);
      if (u == actionToPacked.end() || v == actionToPacked.end()) {
        continue;
      }
      blockList[u->second].push_back(v->second);
    }
    CycleDetection cycleDetection(edges, first, blockList);
    std::vector<std::vector<unsigned>> mutexes;
    cycleDetection.allMinimalCycles(mutexes);
    LOG(4) << "added " << mutexes.size() << " non trivial mutexes";
    // translate minimal cycles to action mutexes
    for (auto &m : mutexes) {
      actionMutexes.emplace_back();
      actionMutexes.back().reserve(m.size());
      for (auto a : m) {
        actionMutexes.back().push_back(packedToAction[a]);
      }
    }
    // // quadratic test for minimality
    // LOG(4) << "computed all cycles";
    // for (size_t i = 0; i < actionMutexes.size() - 1; ++i) {
    //   for (size_t j = i + 1; j < actionMutexes.size(); ++j) {
    //     bool same;
    //     if (actionMutexes[i].size() < actionMutexes[j].size()) {
    //       same = isSubset(actionMutexes[i], actionMutexes[j]);
    //     } else {
    //       same = isSubset(actionMutexes[j], actionMutexes[i]);
    //     }
    //     if (same) {
    //       LOG(0) << "ERR " << i << " " << j;
    //       for (size_t k = 0; k < actionMutexes[i].size(); ++k) {
    //         std::cout << actionMutexes[i][k] << " ";
    //       }
    //       std::cout << std::endl;
    //       for (size_t k = 0; k < actionMutexes[j].size(); ++k) {
    //         std::cout << actionMutexes[j][k] << " ";
    //       }
    //       std::cout << std::endl;
    //     }
    //   }
    // }
    // exit(0);

    addMutexes(actionMutexes);
  }
};
