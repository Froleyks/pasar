#pragma once

#include "search.hpp"

class GreedyBestFirst : public BaseSearch {
private:
  size_t lastGuide = 0;
  inline void
  assignGain(State &state, std::vector<State> &guideStates,
             std::vector<action_t> &appActions,
             std::vector<double> &weights);

public:
  GreedyBestFirst(Problem &problem);
  bool search(std::vector<State> &guideStates, std::vector<action_t> &plan,
              double timeLimit = std::numeric_limits<double>::infinity());
  size_t lastGuideState();
};
