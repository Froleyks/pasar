#pragma once

#include "search.hpp"

class NoSearch {
private:
public:
  size_t firstGuideState = 0;
  NoSearch(Problem &problem __attribute__((unused))) {}
  bool search(std::vector<State> &guideStates __attribute__((unused)),
              std::vector<action_t> &plan __attribute__((unused)),
              double timeLimit __attribute__((unused))) {
    return false;
  }
};
