#pragma once

#include "src/abstraction/sat_based_abstraction.hpp"

class Foreach : public SatBasedAbstraction {
private:
public:
  Foreach(Problem &problem) : SatBasedAbstraction(problem) {
    computeValueSupport();
    atMostOneValue();
    initial();
    assumeGoal();
    preconditions();
    frame();
    effects();

    std::vector<std::pair<action_t, action_t>> mutexes;
    getInterferenceGraph(mutexes);
    addMutexes(f, mutexes);
  }

  inline bool
  solve(std::vector<AbstractPlan::Step> &steps,
        double timeLimit = std::numeric_limits<double>::infinity()) {
    double endTime    = Logger::getTime() + timeLimit;
    unsigned makespan = f.getMakespan();
    if (makespan == 1 && initialMakespan > 1) {
      makespan = f.increaseMakespan(initialMakespan - 1);
    }
    timeLimit   = std::min(endTime - Logger::getTime(), timeLimitPerMakespan);
    bool solved = f.solve(timeLimit);
    if (solved) {
      log(4) << "solved in initial makespan " << makespan;
    }
    while (!solved) {
      int increase =
          std::max((int)(makespan * (makespanIncrease - 1) + 0.1), 1);
      makespan  = f.increaseMakespan(increase);
      timeLimit = std::min(endTime - Logger::getTime(), timeLimitPerMakespan);
      log(5) << "start solving makespan " << makespan;
      solved = f.solve(timeLimit);
    }
    if (solved) {
      log(4) << "solved in makespan " << makespan;
      extractStepSequence(steps, f);
    } else {
      log(4) << "failed in makespan " << makespan;
    }
    return solved;
  }

  inline bool fixStep(State &from, AbstractPlan::Step &actions, State &to,
                      std::vector<action_t> &planForStep) {
    // all steps are correct for each order
    planForStep = actions;
    return true;
  }

  inline void refine(AbstractPlan::Step &step) {
    assert(false);
    return; }
};
