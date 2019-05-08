#include "problem.hpp"

inline void
Problem::parseVariable(std::ifstream &in,
                       std::vector<std::pair<variable_t, bool>> &mapping) {
  std::string name;
  int axiomLevel, numValuesOfVariable;
  in >> name >> axiomLevel >> numValuesOfVariable;
  if (axiomLevel != -1) {
    static bool once = true;
    if (once) {
      once = false;
      log(1) << "Warning: Ignoring axioms";
    }
  }
  if (numValuesOfVariable > maxNumValues) {
    static bool once = true;
    if (once) {
      once = false;
      log(1) << "Warning: number of values too high " << numValuesOfVariable;
    }
  }

  if (numValuesOfVariable == 2) {
    // is prepositional
    mapping.emplace_back(numPreposition++, true);
  } else {
    numValues[numFinDomain] = numValuesOfVariable;
    mapping.emplace_back(numFinDomain++, false);
  }

  // advance stream
  for (int i = 0; i < numValuesOfVariable + 1; ++i) {
    in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
}

inline void
Problem::parseAction(std::ifstream &in,
                     const std::vector<variable_t> &variablePermutation) {
  pre.emplace_back();
  eff.emplace_back();
  disable.emplace_back();
  // TODO: I don't know why this is necessary
  // Advance the sream by one char
  in.get();

  in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  int numPrevailConditions;
  in >> numPrevailConditions;

  // just a lower bound
  pre.back().reserve(numPrevailConditions);
  for (size_t i = 0; i < numPrevailConditions; ++i) {
    int varIndex, value;
    in >> varIndex >> value;
    varIndex = variablePermutation[varIndex];
    pre.back().emplace_back(varIndex, value);
  }

  int numEffects;
  in >> numEffects;
  pre.back().reserve(pre.back().size() + numEffects);
  eff.back().reserve(numEffects);
  for (size_t i = 0; i < numEffects; ++i) {
    int numEffectConditions, varIndex, oldValues;
    int newValue;
    in >> numEffectConditions >> varIndex >> oldValues >> newValue;
    varIndex = variablePermutation[varIndex];
    if (numEffectConditions) {
      static bool once = true;
      if (once) {
        once = false;
        log(1) << "Warning: Ignoring conditional effects. Parsing might break!";
      }
    }
    if (oldValues != -1) {
      pre.back().emplace_back(varIndex, oldValues);
      // TODO: At Most One value
      // is an explicit at most one better?

      disable.back().emplace_back(varIndex, oldValues);
    } else {
      for (size_t other = 0; other < numValues[varIndex]; ++other) {
        if (other == newValue) {
          continue;
        }
        disable.back().emplace_back(varIndex, other);
      }
    }
    eff.back().emplace_back(varIndex, newValue);
  }
  int operatorCost;
  in >> operatorCost;
  static bool once = true;
  if (once) {
    once = false;
    log(2) << "Warning: Ignoring metric";
  }

  // TODO does this help?
  // sort vectors for more cache efficient access
  std::sort(pre.back().begin(), pre.back().end());
  std::sort(eff.back().begin(), eff.back().end());
  std::sort(disable.back().begin(), disable.back().end());
}

inline void
Problem::parseMutex(std::ifstream &in,
                    std::vector<std::pair<variable_t, value_t>> &mutex,
                    const std::vector<variable_t> &variablePermutation) {
  unsigned numVarsInMutex;
  in >> numVarsInMutex;
  mutex.reserve(numVarsInMutex);
  for (size_t i = 0; i < numVarsInMutex; ++i) {
    int variable;
    int value;
    in >> variable >> value;
    variable = variablePermutation[variable];
    mutex.emplace_back(variable, value);
  }
}

void Problem::load(std::string file) {
  enum PareseState { preamble, variable, mutex, initial, goal, action, axiom };
  PareseState state = preamble;
  std::ifstream in(file);
  if (!in.good()) {
    throw std::runtime_error("file error");
  }
  std::string token;

  // reorder variables: finDomain, preposition
  std::vector<std::pair<variable_t, bool>> mapping;
  std::vector<variable_t> variablePermutation;
  while (in.good()) {
    switch (state) {
    case preamble: {
      in >> token;
      if (token == "end_metric") {
        state = variable;
      }
    } break;
    case variable: {
      in >> numVariables;
      if (numVariables > maxNumVariables) {
        log(1) << "Warning: Too many Variables";
      }
      mapping.reserve(numVariables);
      numValues.reserve(numVariables);
      for (size_t i = 0; i < numVariables; ++i) {
        in >> token;
        assert(token == "begin_variable");
        parseVariable(in, mapping);
        in >> token;
        assert(token == "end_variable");
      }
      variablePermutation.reserve(numVariables);
      for (size_t i = 0; i < numVariables; ++i) {
        variablePermutation.push_back(mapping[i].first);
        // first finDomain then preposition
        if (mapping[i].second) {
          variablePermutation.back() += numFinDomain;
        }
      }
      state                      = mutex;
      CompactState::numFinDomain = numFinDomain;
    } break;
    case mutex: {
      unsigned numMutexes;
      in >> numMutexes;
      mutexes.resize(numMutexes);
      for (size_t i = 0; i < numMutexes; ++i) {
        in >> token;
        assert(token == "begin_mutex_group");
        parseMutex(in, mutexes[i], variablePermutation);
        in >> token;
        assert(token == "end_mutex_group");
      }
      state = initial;
    } break;
    case initial: {
      in >> token;
      assert(token == "begin_state");
      initialState.resize(numVariables);
      for (size_t i = 0; i < numVariables; ++i) {
        int value;
        in >> value;
        initialState[variablePermutation[i]] = value;
      }
      in >> token;
      assert(token == "end_state");
      state = goal;
    } break;
    case goal: {
      in >> token;
      assert(token == "begin_goal");
      int numGoals;
      in >> numGoals;
      goalState.resize(numVariables, unassigned);
      for (size_t i = 0; i < numGoals; ++i) {
        int varIndex, value;
        in >> varIndex >> value;
        varIndex            = variablePermutation[varIndex];
        goalState[varIndex] = value;
      }
      in >> token;
      assert(token == "end_goal");
      state = action;
    } break;
    case action: {
      in >> numActions;
      lastOriginalAction = numActions - 1;
      if (maxNumActions > numActions) {
        log(1) << "Warning: Too many actions";
      }
      pre.reserve(numActions);
      eff.reserve(numActions);
      disable.reserve(numActions);
      for (size_t i = 0; i < numActions; ++i) {
        in >> token;
        assert(token == "begin_operator");
        parseAction(in, variablePermutation);
        in >> token;
        assert(token == "end_operator");
      }
      state = axiom;
    } break;
    case axiom: {
      in >> token;
      if (token != "0") {
        static bool once = true;
        if (once) {
          once = false;
          log(1) << "Warning: Ignoring axiom";
        }
      }
    } break;
    }
  }
}
