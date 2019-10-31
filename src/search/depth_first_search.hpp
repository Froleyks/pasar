#pragma once

#include "search.hpp"

class DepthFirstSearch : public BaseSearch {
private:
public:
  DepthFirstSearch(Problem &problem);
  // guide sates are ignored
  bool search(std::vector<State> &guideStates, std::vector<action_t> &plan,
              double timeLimit = std::numeric_limits<double>::infinity(),
              int nodeLimit    = -1);

  size_t lastGuideState() { return 0; }
};
