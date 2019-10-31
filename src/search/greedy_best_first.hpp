#pragma once

#include "search.hpp"

class GreedyBestFirst : public BaseSearch {
private:
  inline void
  updateGuideState(State &state, std::vector<State> &guideStates,
                   std::vector<action_t> &plan,
                   std::vector<std::pair<size_t, size_t>> &milestones,
                   std::vector<char> &reachedGuideHint);

public:
  GreedyBestFirst(Problem &problem);
  bool search(std::vector<State> &guideStates, std::vector<action_t> &plan,
              double timeLimit = std::numeric_limits<double>::infinity(),
              int nodeLimit    = -1);
};
