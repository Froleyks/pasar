#pragma once

#include "search.hpp"

class GreedyBestFirst : public BaseSearch {
private:
  inline void updateGuideState(const std::vector<State> &guideStates,
                               std::vector<action_t> &plan);

  State state;
  CompactState compactState;
  StateHashSet knownStates;
  std::vector<Assignment> changeHistory;
  WeightedActionSet appActions;

  // before plan index <scond> <first> was the highest reached guideStates
  std::vector<std::pair<size_t, size_t>> milestones{{-1, 0}};

  // used to indicate if there is an action in the set of applicable actions
  // that might reach this state
  std::vector<char> reachedGuideHint;

public:
  GreedyBestFirst(Problem &problem);
  bool continueSearch(const std::vector<State> &guideStates,
                             std::vector<action_t> &plan);

  void reset(const std::vector<State> &guideStates);
  bool search(std::vector<State> &guideStates, std::vector<action_t> &plan);
};
