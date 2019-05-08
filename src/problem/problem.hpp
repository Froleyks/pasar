#pragma once

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "src/type_defs.hpp"
#include "tools/logger.hpp"

#include "state_hash.hpp"

static constexpr size_t maxNumVariables =
    std::numeric_limits<variable_t>::max() + 1;
static constexpr value_t unassigned  = std::numeric_limits<value_t>::max();
static constexpr size_t maxNumValues = std::numeric_limits<value_t>::max();
static constexpr size_t maxNumActions =
    std::numeric_limits<action_t>::max() + 1;

// Parse sas problem file
// This should be Singleton
class Problem {
private:
  variable_t numPreposition = 0;
  variable_t numFinDomain   = 0;

  // mapping is used to sort the variables into prepositional and final domain
  // variables
  inline void parseVariable(std::ifstream &in,
                            std::vector<std::pair<variable_t, bool>> &mapping);

  // after loading every variable has undergone the permutation. The action
  // indices are not changed.
  inline void parseAction(std::ifstream &in,
                          const std::vector<variable_t> &variablePermutation);

  inline void parseMutex(std::ifstream &in,
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

  // partial assignment
  using Assignment = std::vector<std::pair<variable_t, value_t>>;

  // each action has one index in pre, eff, and disable
  // could be stored continuously in memory for better caching
  std::vector<Assignment> pre;
  std::vector<Assignment> eff;
  // values to disable for finDomain variables effected by action
  // used for sat encodings
  std::vector<Assignment> disable;

  // additional knowledge
  std::vector<std::vector<std::pair<variable_t, value_t>>> mutexes;

  // must be complete
  State initialState;
  // may be partial
  // expanded for faster random access
  State goalState;

  void load(std::string file);
};
