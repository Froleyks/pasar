#pragma once

extern "C" {
#include "sat/ipasir.h"
}

#include "src/problem/problem.hpp"
#include "tools/logger.hpp"
#include <limits>

// this capsules the ipasir interface
// clauses added will be added to all past and future steps
class Formula {
private:
  bool withSolver_;
  void *solver;
  int numVariables       = 0;
  size_t newClausesBegin = 0;
  int currentStep        = -1;
  // makestep x variable x value
  size_t initialClausesIndex = 0;

  struct Variable {
    bool isEnd;
    bool polarity;
    bool isState;
    size_t t;
    action_t i;
    value_t v;
    bool isToggle;
    Variable(bool p_isEnd = true, bool p_polarity = false,
             bool p_isState = false, size_t p_t = 0, action_t p_i = 0,
             value_t p_v = 0, bool p_isToggle = false)
        : isEnd(p_isEnd), polarity(p_polarity), isState(p_isState), t(p_t),
          i(p_i), v(p_v), isToggle(p_isToggle) {}
  };

  std::vector<Variable> clauses;
  Assignment assumptions;

  void activateAssumptions() {
    for (auto [variable, value] : assumptions) {
      ipasir_assume(solver, state.back()[variable][value]);
    }
    // activate toggles
    for (size_t variable = 0; variable < toggle.size(); ++variable) {
      for (size_t value = 0; value < toggle[variable].size(); ++value) {
        // assume false for all except last
        for (size_t i = 0; i < toggle[variable][value].size() - 1; ++i) {
          ipasir_assume(solver, -toggle[variable][value][i]);
        }
        // assume true for last
        ipasir_assume(solver, toggle[variable][value].back());
      }
    }
  }

  inline void add(Variable &v) {
    int lit = 0;
    if (!v.isEnd) {
      if (v.isState) {
        lit = state[v.t][v.i][v.v];
      } else {

        if (v.isToggle) {
          lit = toggle[v.i][v.v][v.t];
        } else {
          lit = action[v.t][v.i];
        }
      }
      if (!v.polarity) {
        lit = -lit;
      }
    }
    ipasir_add(solver, lit);
  }

  void addClausesForNewStep() {
    for (size_t i = 0; i < clauses.size(); ++i) {
      if (clauses[i].isToggle) {
        const size_t iteration = toggle[clauses[i].i][clauses[i].v].size() - 1;
        if (clauses[i].t == iteration) {
          // is current
          add(clauses[i]);
        }
      } else {
        clauses[i].t++;
        add(clauses[i]);
      }
    }
  }

  // only new clauses
  // known clauses are added in increase makespan
  void addClausesForAllSteps() {
    if (newClausesBegin == clauses.size()) {
      return;
    }
    std::vector<Variable> new_clauses(
        clauses.begin() + static_cast<long>(newClausesBegin), clauses.end());
    newClausesBegin = clauses.size();
    assert(new_clauses.back().isEnd);
    for (int step = 0; step < currentStep + 1; ++step) {
      for (unsigned i = 0; i < new_clauses.size(); ++i) {
        add(new_clauses[i]);
        if (!new_clauses[i].isToggle) {
          new_clauses[i].t--;
        }
      }
    }
  }

public:
  // time x variable x value
  std::vector<std::vector<std::vector<int>>> state;
  // time x action
  std::vector<std::vector<int>> action;
  // variable x value x iteration
  std::vector<std::vector<std::vector<int>>> toggle;

  Formula(const Problem &problem, bool togglable = false,
          bool withSolver                                = true,
          const std::vector<Variable> &additionalClauses = {})
      : withSolver_(withSolver) {
    if (withSolver) {
      solver = ipasir_init();

      // glucose fix
      ipasir_set_learn(solver, NULL, 0, NULL);

      log(3) << "Using the incremental SAT solver" << ipasir_signature();
    }

    // set state variables
    state.emplace_back(problem.numVariables);
    // all finite domain variables
    for (size_t i = 0; i < problem.numValues.size(); ++i) {
      state[0][i].resize(problem.numValues[i]);
      for (size_t v = 0; v < problem.numValues[i]; ++v) {
        state[0][i][v] = ++numVariables;
      }
    }
    // prepostions
    for (size_t i = problem.numValues.size(); i < problem.numVariables; ++i) {
      // no dual-rail encoding for binary values
      ++numVariables;
      state[0][i] = {numVariables, -numVariables};
    }

    // set action variables
    action.emplace_back(problem.numActions);
    for (size_t i = 0; i < action[0].size(); ++i) {
      action[0][i] = ++numVariables;
    }

    if (togglable) {
      toggle.resize(state[0].size());
      for (variable_t variable = 0; variable < state[0].size(); ++variable) {
        toggle[variable].resize(state[0][variable].size());
        for (value_t value = 0; value < state[0][variable].size(); ++value) {
          toggle[variable][value] = {++numVariables};
        }
      }
    }

    // TODO the last set of action vars is not needed
    increaseMakespan();

    // at least one value
    // is not necessary for the fixed initial sate
    for (variable_t variable = 0; variable < state[0].size(); ++variable) {
      if (state[0][variable].size() > 2) {
        for (value_t value = 0; value < state[0][variable].size(); ++value) {
          addS(variable, value, true, true);
        }
        close();
      }
    }

    // mutex which are generated by sas transform of fastdownward
    for (const auto &mutex : problem.mutexes) {
      for (size_t i = 0; i < mutex.size() - 1; ++i) {
        for (size_t j = i + 1; j < mutex.size(); ++j) {
          addS(mutex[i].first, mutex[i].second, false, true);
          addS(mutex[j].first, mutex[j].second, false, true);
          close();
        }
      }
    }
    initialClausesIndex = clauses.size();
    clauses.insert(clauses.begin(), additionalClauses.begin(),
                   additionalClauses.end());
  }

  ~Formula() {
    if (withSolver_) {
      ipasir_release(solver);
    }
  }

  // add sat variables for new actions for all steps
  inline void addVarsForActions(size_t numNewActions) {
    // add for current step
    action.back().reserve(action.back().size() + numNewActions);
    for (size_t a = 0; a < numNewActions; ++a) {
      action.back().push_back(++numVariables);
    }

    // add for all previous steps
    for (size_t step = 0; step < action.size() - 1; ++step) {
      action[step].reserve(action[step].size() + numNewActions);
      for (size_t a = 0; a < numNewActions; ++a) {
        action[step].push_back(++numVariables);
      }
    }
  }

  size_t getMakespan() { return static_cast<size_t>(currentStep) + 1; }

  size_t increaseMakespan(unsigned steps = 1) {
    for (unsigned s = 0; s < steps; ++s) {
      // set state variables
      // copy everything
      state.push_back(state.back());
      for (size_t variable = 0; variable < state.back().size(); ++variable) {
        for (size_t value = 0; value < state.back()[variable].size(); ++value) {
          if (state.back()[variable][value] > 0) {
            state.back()[variable][value] = ++numVariables;
          } else {
            // preposition optimisation
            assert(state.back()[variable].size() == 2 && value == 1);
            state.back()[variable][value] = -state.back()[variable][value - 1];
          }
        }
      }

      // set action variables
      action.emplace_back(action.back().size());
      for (size_t i = 0; i < action.back().size(); ++i) {
        action.back()[i] = ++numVariables;
      }

      addClausesForNewStep();

      currentStep++;
    }

    return static_cast<size_t>(currentStep) + 1;
  }

  // assuming the goal
  inline void assumeAtLast(int variable, unsigned value) {
    assumptions.emplace_back(variable, value);
  }

  inline void addInitialStateAtom(variable_t variable, value_t value,
                                  bool polarity = true) {
    ipasir_add(solver, (polarity ? 1 : -1) * state[0][variable][value]);
    ipasir_add(solver, 0);
  }

  // adds new toggle, old will be disabled
  inline void iterateToggle(variable_t variable, value_t value) {
    toggle[variable][value].push_back(++numVariables);
  }

  // clauses added will be added to all past and future steps
  inline void addS(int variable, unsigned value, bool polarity = true,
                   bool next = false) {

    if (next) {
      clauses.emplace_back(false, polarity, true, currentStep + 1, variable,
                           value);
    } else {
      clauses.emplace_back(false, polarity, true, currentStep, variable, value);
    }
  }

  inline void addA(action_t a, bool polarity = true, bool next = false) {
    if (next) {
      clauses.emplace_back(false, polarity, false, currentStep + 1, a);
    } else {
      clauses.emplace_back(false, polarity, false, currentStep, a);
    }
  }

  // toggles are independent of the time step
  inline void addT(variable_t variable, unsigned value, bool polarity = true) {
    const size_t iteration = toggle[variable][value].size() - 1;
    clauses.emplace_back(false, polarity, false, iteration, variable, value,
                         true);
  }

  inline void close() { clauses.emplace_back(); }

  // assume goal and try to solve
  bool solve(double timeLimit = std::numeric_limits<double>::infinity()) {
    log(5) << "start sat solver";
    addClausesForAllSteps();
    activateAssumptions();
    double endTime = Logger::getTime() + timeLimit;
    ipasir_set_terminate(solver, &endTime, [](void *time) {
      return static_cast<int>(Logger::getTime() >
                              *static_cast<double *>(time));
    });
    int satRes = ipasir_solve(solver);
    return satRes == 10;
  }

  value_t getStateValue(variable_t variable, size_t t) const {
    for (value_t value = 0; value < state[t][variable].size(); ++value) {
      int lit = state[t][variable][value];
      if (lit > 0) {
        // TODO not important values
        if (ipasir_val(solver, lit) >= 0) {
          return value;
        }
      } else {
        if (ipasir_val(solver, -lit) <= 0) {
          return value;
        }
      }
    }
    assert(false);
    return unassigned;
  }

  bool getActionValue(action_t a, size_t t) const {
    // not important variables should be false, a minimum of actions is
    // preferred
    return ipasir_val(solver, action[t][a]) > 0;
  }

  void getClauses(std::vector<Variable> &c) {
    c = std::vector<Variable>(
        clauses.begin() + static_cast<long>(newClausesBegin), clauses.end());
  }
};
