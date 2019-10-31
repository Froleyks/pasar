#pragma once

#include "src/abstraction/unrelaxed.hpp"

class Foreach : public Unrelaxed {
public:
  Foreach(Problem &problem, bool cegar = false) : Unrelaxed(problem) {
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
    addMutexes(edgeList);
  }
};
