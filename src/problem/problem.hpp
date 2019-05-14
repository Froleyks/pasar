#pragma once

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "src/type_defs.hpp"
#include "tools/logger.hpp"

// Parse sas problem file
// This should be Singleton
class Problem {
private:
  static constexpr size_t maxNumVariables =
      std::numeric_limits<variable_t>::max() + 1;
  static constexpr size_t maxNumValues  = std::numeric_limits<value_t>::max();
  static constexpr size_t maxNumActions = std::numeric_limits<action_t>::max();

  variable_t numPreposition = 0;
  variable_t numFinDomain   = 0;

  // mapping is used to sort the variables into prepositional and finite domain
  // variables
  void parseVariable(std::ifstream &in,
                     std::vector<std::pair<variable_t, bool>> &mapping);

  // after loading every variable has undergone the permutation. The action
  // indices are not changed.
  void parseAction(std::ifstream &in,
                   const std::vector<variable_t> &variablePermutation);

  void parseMutex(std::ifstream &in,
                  std::vector<std::pair<variable_t, value_t>> &mutex,
                  const std::vector<variable_t> &variablePermutation);

public:
  Problem() {}
  Problem(std::string file) { load(file); }

  // variables
  size_t numVariables = 0;
  // only for finDomain
  std::vector<value_t> numValues;

  // actions
  size_t numActions;
  size_t lastOriginalAction;

  // each action has one index in pre and eff
  // could be stored continuously in memory for better caching (would waste memory)
  std::vector<Assignment> pre;
  std::vector<Assignment> eff;

  // each learned action has a plan to back it up
  std::vector<std::vector<action_t>> witness;

  // additional knowledge
  std::vector<std::vector<std::pair<variable_t, value_t>>> mutexes;

  // must be complete
  State initialState;
  // partial
  Assignment goal;
  // expanded for faster random access
  State goalState;

  void load(std::string file);

  void removeLearnedActions(std::vector<action_t> &planWithLearned);

  static std::string planToString(std::string file,
                                  std::vector<action_t> &plan);
};
