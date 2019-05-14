#pragma once

#include "abstraction.hpp"

class SatBasedAbstraction : public BaseAbstraction {
private:
protected:
  unsigned initialMakespan    = 5;
  double makespanIncrease     = 1.2;
  double timeLimitPerMakespan = std::numeric_limits<double>::infinity();

  void addMutexes(Formula &f,
                  const std::vector<std::pair<action_t, action_t>> &mutexes) {
    for (auto [a1, a2] : mutexes) {
      // -a1 v -a2
      f.addA(a1, false);
      f.addA(a2, false);
      f.close();
    }
    log(3) << "added " << mutexes.size() << " no interference clauses";
  }

  void extractStepSequence(std::vector<AbstractPlan::Step> &stepSequence,
                           Formula &f) {
    stepSequence.resize(f.action.size() - 1);
    for (size_t t = 0; t < f.action.size() - 1; ++t) {
      for (action_t a = 0; a < f.action[t].size(); ++a) {
        if (f.getActionValue(a, t)) {
          stepSequence[t].push_back(a);
        }
      }
    }
  }

public:
  static const bool isSatBased = true;
  Formula f;

  SatBasedAbstraction(Problem &problem)
      : BaseAbstraction(problem), f(problem) {}

  void setInitialMakespan(unsigned initialMakespan) {
    this->initialMakespan = initialMakespan;
  }

  void setMakespanIncrease(double makespanIncrease) {
    this->makespanIncrease = makespanIncrease;
  }

  void setTimeLimitPerMakespan(double timeLimitPerMakespan) {
    this->timeLimitPerMakespan = timeLimitPerMakespan;
  }

  // SAT clauses
  inline void atMostOneValue() {
    // only for finite domain variables
    for (variable_t variable = 0; variable < problem.numValues.size();
         ++variable) {
      for (variable_t i = 0; i < problem.numValues[variable] - 1; ++i) {
        for (variable_t j = i + 1; j < problem.numValues[variable]; ++j) {
          // -(variable,i) v -(variable, j)
          f.addS(variable, i, false);
          f.addS(variable, j, false);
          f.close();
        }
      }
    }
  }

  inline void initial() {
    // finite domain
    for (variable_t variable = 0; variable < problem.numValues.size();
         ++variable) {
      for (value_t value = 0; value < f.state[0][variable].size(); ++value) {
        if (problem.initialState[variable] == value) {
          f.addInitialStateAtom(variable, value, true);
        } else {
          f.addInitialStateAtom(variable, value, false);
        }
      }
    }
    // prepostions
    for (variable_t variable = problem.numValues.size();
         variable < problem.numVariables; ++variable) {
      for (value_t value = 0; value < f.state[0][variable].size(); ++value) {
        if (problem.initialState[variable] == value) {
          f.addInitialStateAtom(variable, value, true);
          continue;
        } else {
          f.addInitialStateAtom(variable, value, false);
          continue;
        }
      }
    }
  }

  inline void assumeGoal() {
    for (auto [variable, value] : problem.goal) {
      f.assumeAtLast(variable, value);
    }
  }

  inline void atLeastOneAction() {
    for (action_t a = 0; a < f.action[0].size(); ++a) {
      f.addA(a);
    }
    f.close();
  }

  inline void frame() {
    for (variable_t variable = 0; variable < f.state[0].size(); ++variable) {
      for (value_t value = 0; value < f.state[0][variable].size(); ++value) {
        // p_t+1 => p_t v s in supp(p): s_t
        f.addS(variable, value, false, true);
        f.addS(variable, value, true);
        for (auto a : valueSupport[variable][value]) {
          f.addA(a);
        }
        f.close();
      }
    }
  }

  inline void preconditions() {
    for (action_t a = 0; a < f.action[0].size(); ++a) {
      // a_t => pre(a)_t
      for (auto [variable, value] : problem.pre[a]) {
        f.addA(a, false);
        f.addS(variable, value);
        f.close();
      }
    }
  }

  inline void effects() {
    for (action_t a = 0; a < f.action[0].size(); ++a) {
      // a_t => eff(a)_t+1
      for (auto [variable, value] : problem.eff[a]) {
        f.addA(a, false);
        f.addS(variable, value, true, true);
        f.close();
      }
    }
  }

  inline void atMostOneAction() {
    for (action_t i = 0; i < f.action[0].size() - 1; ++i) {
      for (action_t j = i + 1; j < f.action[0].size(); ++j) {
        // -a_i v -a_j
        f.addA(i, false);
        f.addA(j, false);
        f.close();
      }
    }
  }
};
