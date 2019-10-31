#pragma once

#include "abstraction.hpp"

class SatBasedAbstraction : public BaseAbstraction {
private:
protected:
  int conflictLimitPerMakespan_ = -1;
  double timeLimitPerMakespan_  = std::numeric_limits<double>::infinity();
  unsigned initialMakespan_     = 5;
  double makespanIncrease_      = 1.2;

  action_t firstActionToAdd = 0;
  size_t firstNoGoodToAdd   = 0;

  inline void
  addMutexes(const std::vector<std::pair<action_t, action_t>> &mutexes) {
    numRefineSteps++;
    numRefinements += mutexes.size();
    for (auto [a1, a2] : mutexes) {
      sumRefineLength += 2;
      // -a1 v -a2
      f.addA(a1, false);
      f.addA(a2, false);
      f.close();
    }
    LOG(4) << "added " << mutexes.size() << " no interference clauses";
  }

  inline void addMutexes(const std::vector<std::vector<action_t>> &mutexes) {
    numRefineSteps++;
    numRefinements += mutexes.size();
    for (auto &m : mutexes) {
      sumRefineLength += m.size();
      // -a1 v -a2 v ...
      for (auto a : m) {
        f.addA(a, false);
      }
      f.close();
    }
    LOG(4) << "added " << mutexes.size() << " mutexes";
  }

  inline void addNewActions() {
    if (problem_.numActions == firstActionToAdd) {
      return;
    }
    f.addVarsForActions(problem_.numActions - firstActionToAdd);

    std::vector<std::pair<variable_t, value_t>> updatedSupports;
    updateValueSupport(firstActionToAdd, updatedSupports);

    // remove duplicates
    std::sort(updatedSupports.begin(), updatedSupports.end());
    updatedSupports.erase(
        std::unique(updatedSupports.begin(), updatedSupports.end()),
        updatedSupports.end());

    preconditions(firstActionToAdd);
    effects(firstActionToAdd);

    for (auto [variable, value] : updatedSupports) {
      updateFrame(variable, value);
    }
    firstActionToAdd = static_cast<action_t>(problem_.numActions);
  }

  inline void
  extractStepSequence(std::vector<AbstractPlan::Step> &stepSequence) {
    stepSequence.resize(f.action.size() - 1);
    for (size_t t = 0; t < f.action.size() - 1; ++t) {
      for (action_t a = 0; a < f.action[t].size(); ++a) {
        if (f.getActionValue(a, t)) {
          stepSequence[t].push_back(a);
          if (a > problem_.lastOriginalAction) {
            LOG(3) << "Learned action in abstract plan " << a;
          }
        }
      }
    }
  }

  /// SAT clauses
  inline void atMostOneValue() {
    // only for finite domain variables
    for (variable_t variable = 0; variable < problem_.numValues.size();
         ++variable) {
      for (variable_t i = 0; i < problem_.numValues[variable] - 1; ++i) {
        for (variable_t j = static_cast<variable_t>(i + 1);
             j < problem_.numValues[variable]; ++j) {
          // -(variable,i) v -(variable, j)
          f.addS(variable, i, false, true);
          f.addS(variable, j, false, true);
          f.close();
        }
      }
    }
  }

  inline void initial() {
    // finite domain
    for (variable_t variable = 0; variable < problem_.numValues.size();
         ++variable) {
      for (value_t value = 0; value < f.state[0][variable].size(); ++value) {
        if (problem_.initialState[variable] == value) {
          f.addInitialStateAtom(variable, value, true);
        } else {
          f.addInitialStateAtom(variable, value, false);
        }
      }
    }
    // prepostions
    for (variable_t variable =
             static_cast<variable_t>(problem_.numValues.size());
         variable < problem_.numVariables; ++variable) {
      for (value_t value = 0; value < f.state[0][variable].size(); ++value) {
        if (problem_.initialState[variable] == value) {
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
    for (auto [variable, value] : problem_.goal) {
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

  inline void toggleFrame() {
    for (variable_t variable = 0; variable < f.state[0].size(); ++variable) {
      for (value_t value = 0; value < f.state[0][variable].size(); ++value) {
        // p_t+1 => p_t v s in supp(p): s_t v T_0
        f.addS(variable, value, false, true);
        f.addS(variable, value, true);
        for (auto a : valueSupport[variable][value]) {
          f.addA(a);
        }
        f.addT(variable, value, false);
        f.close();
      }
    }
  }

  inline void updateFrame(variable_t variable, value_t value) {
    f.iterateToggle(variable, value);
    // p_t+1 => p_t v s in supp(p): s_t v T_0
    f.addS(variable, value, false, true);
    f.addS(variable, value, true);
    for (auto a : valueSupport[variable][value]) {
      f.addA(a);
    }
    f.addT(variable, value, false);
    f.close();
  }

  inline void preconditions(action_t firstAction = 0) {
    for (action_t a = firstAction; a < f.action[0].size(); ++a) {
      // a_t => pre(a)_t
      for (auto [variable, value] : problem_.pre[a]) {
        f.addA(a, false);
        f.addS(variable, value);
        f.close();
      }
    }
  }

  inline void effects(action_t firstAction = 0) {
    for (action_t a = firstAction; a < f.action[0].size(); ++a) {
      // a_t => eff(a)_t+1
      for (auto [variable, value] : problem_.eff[a]) {
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

public:
  static const bool isSatBased = true;
  Formula f;

  SatBasedAbstraction(Problem &problem, bool togglable = false)
      : BaseAbstraction(problem), f(problem, togglable) {}

  inline void setConflictLimitPerMakespan(int conflictLimitPerMakespan) {
#ifndef GLUCOSE
    // only glucose supports this setting
    assert(conflictLimitPerMakespan < 0);
#endif
    conflictLimitPerMakespan_ = conflictLimitPerMakespan;
  }

  inline void setTimeLimitPerMakespan(double timeLimitPerMakespan) {
    timeLimitPerMakespan_ = timeLimitPerMakespan;
    if (timeLimitPerMakespan < 0) {
      timeLimitPerMakespan_ = std::numeric_limits<double>::infinity();
    }
  }

  inline void setInitialMakespan(unsigned initialMakespan) {
    initialMakespan_ = initialMakespan;
  }

  inline void setMakespanIncrease(double makespanIncrease) {
    makespanIncrease_ = makespanIncrease;
  }

  // return 0: unsolevd 1: solved in increased makespan 2: solved in initial
  // makespan
  inline int solve(std::vector<AbstractPlan::Step> &steps,
                   double timeLimit = std::numeric_limits<double>::infinity()) {
    LOG(4) << "start solving abstraction";

    double endTime = Logger::getTime() + timeLimit;
    addNewActions();
    size_t makespan = f.getMakespan();
    if (makespan == 1 && initialMakespan_ > 1) {
      makespan = f.increaseMakespan(initialMakespan_ - 1);
    }
    timeLimit   = std::min(endTime - Logger::getTime(), timeLimitPerMakespan_);
    bool solved = f.solve(timeLimit, conflictLimitPerMakespan_);
    if (solved) {
      LOG(4) << "solved abstraction in initial makespan " << makespan;
      extractStepSequence(steps);
      return 2;
    }
    while (!solved) {
      unsigned increase = std::max(
          static_cast<unsigned>(
              (makespanIncrease_ - 1) * static_cast<double>(makespan) + 0.1),
          1u);
      makespan = f.increaseMakespan(increase);

      timeLimit = std::min(endTime - Logger::getTime(), timeLimitPerMakespan_);
      LOG(5) << "start solving makespan " << makespan;
      solved = f.solve(timeLimit, conflictLimitPerMakespan_);
      if (endTime <= Logger::getTime()) {
        LOG(5) << "exceeded total time limit";
        break;
      }
    }
    if (solved) {
      LOG(4) << "solved abstraction in makespan " << makespan;
      extractStepSequence(steps);
    } else {
      LOG(4) << "failed in makespan " << makespan << Logger::getTime() - endTime
             << " over time limit";
    }
    return solved;
  }
};
