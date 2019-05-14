#pragma once

#include "search.hpp"

class GreedyBestFirst : public BaseSearch {
private:
  inline void updateGuideState(State &state, std::vector<State> &guideStates,
                               std::vector<action_t> &plan,
                               std::vector<std::pair<int, size_t>> &milestones);

public:
  GreedyBestFirst(Problem &problem);
  bool search(std::vector<State> &guideStates, std::vector<action_t> &plan,
              double timeLimit = std::numeric_limits<double>::infinity());
};
