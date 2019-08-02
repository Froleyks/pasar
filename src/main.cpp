#include <iostream>

#include "tools/logger.hpp"
#include "tools/parameter_processor.hpp"

#include "src/pasar.hpp"
#include "src/problem/problem.hpp"
#include "src/problem/validate_plan.hpp"

#include "abstraction/cycle_break.hpp"
#include "abstraction/fast_forward.hpp"
#include "abstraction/foreach.hpp"
#include "abstraction/leaf_reduction.hpp"
#include "abstraction/no_abstraction.hpp"
#include "abstraction/true_exist.hpp"

#include "src/search/depth_first_search.hpp"
#include "src/search/greedy_best_first.hpp"
#include "src/search/no_search.hpp"

void outputPlan(std::string &outputFile, std::string &problemFile,
                std::vector<action_t> &plan) {
  std::string printPlan = Problem::planToString(problemFile, plan);
  std::ofstream out(outputFile);
  out << printPlan;
  out.close();
}

void addDefaults(ParameterProcessor &params) {
  params.addDefault("s", "0",
                    "used abstraction and search schedule\n"
                    "\t0: portfolio\n"
                    "\t1: pasar true exist fallback foreach\n"
                    "\t2: pasar true exist\n"
                    "\t3: pasar foreach\n"
                    "\t4: pasar leaf reduction\n"
                    "\t5: pasar cycle break\n"
                    "\t6: pure pasar true exist fallback foreach\n"
                    "\t7: pure pasar foreach\n"
                    "\t8: pure pasar cycle break\n"
                    "\t9: encoding true exist\n"
                    "\t10: encoding foreach\n"
                    "\t11: fast forward portfolio\n"
                    "\t12: greedy best first search\n"
                    "\t13: depth first search\n"
                    "\t14: restart search");
  params.addDefault("spar", "2",
                    "sparsification\n"
                    "\t0: complete states\n"
                    "\t1: partial states\n"
                    "\t2: action elimination");
  params.addDefault(
      "cont", "0",
      "contraction\n"
      "\t1: additional actions are learned and fewer guide states are used");

  // search
  params.addDefault("seed", "42",
                    "seed used for random tie breaking during search");
  params.addDefault(
      "gdf", "1.2",
      "gainDecayFactor ≥ 1\n"
      "\thigh gainDecayFactor ⇒ guideStates further from the goal\n"
      "\thave less impact on the direction of the search");

  // abstraction
  params.addDefault("im", "5", "initial makespan");
  params.addDefault("mi", "1.2", "makespan increase");
  params.addDefault("mt", "15",
                    "\ttime limit per makespan\n"
                    "\t0: disabled");
  params.addDefault("sml", "40",
                    "same makespan limit\n"
                    "\tif enough abstractions are solved in the same makespan "
                    "the cegar approach is aborted\n"
                    "\t0: disable");
}

bool runSchedule(const ParameterProcessor &params, Pasar &solver,
                 Problem &problem, int schedule, std::vector<action_t> &plan) {
  bool solved                = false;
  unsigned sameMakespanLimit = static_cast<unsigned>(params.getInt("sml"));
  unsigned sameMakespanCount = 0;
  switch (schedule) {
  case 0: {
    LOG(2) << "running schedule " << schedule << " portfolio";
    solver.setAbstractionTimeout(0);
    solver.setSearchTimeout(100);
    NoAbstraction dummy(problem);
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    solved = solver.findPlan(dummy, search, plan, sameMakespanCount);
    if (!solved) {
      TrueExist abstraction(problem, true);
      solver.setSearchTimeout(0.2);
      solver.setAbstractionTimeout(-1);
      while (!solved) {
        if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
          abstraction.addAllInterferances();
        }
        solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
      }
    }
    break;
  }
  case 1: {
    LOG(2) << "running schedule " << schedule
           << " pasar true exist fallback foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0.2);
    TrueExist abstraction(problem, true);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.addAllInterferances();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 2: {
    LOG(2) << "running schedule " << schedule << " pasar true exist";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0.2);
    TrueExist abstraction(problem, true);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.refine();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 3: {
    LOG(2) << "running schedule " << schedule << " pasar foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0.2);
    Foreach abstraction(problem, true);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.refine();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 4: {
    LOG(2) << "running schedule " << schedule << " pasar leaf reduction";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0.2);
    LeafReduction abstraction(problem, true);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.refine();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 5: {
    LOG(2) << "running schedule " << schedule << " pasar cycle break";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0.2);
    CycleBreak abstraction(problem);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.refine();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 6: {
    LOG(2) << "running schedule " << schedule
           << " pure pasar true exist fallback foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    TrueExist abstraction(problem, true);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    NoSearch search(problem);
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.addAllInterferances();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 7: {
    LOG(2) << "running schedule " << schedule << " pure pasar foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    Foreach abstraction(problem, true);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    NoSearch search(problem);
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.refine();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 8: {
    LOG(2) << "running schedule " << schedule << " pure pasar cycle break";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    CycleBreak abstraction(problem);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    NoSearch search(problem);
    while (!solved) {
      if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
        abstraction.refine();
      }
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  case 9: {
    LOG(2) << "running schedule " << schedule << " encoding true exist";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    TrueExist abstraction(problem);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    NoSearch search(problem);
    solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    break;
  }
  case 10: {
    LOG(2) << "running schedule " << schedule << " encoding foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    Foreach abstraction(problem);
    abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
    abstraction.setMakespanIncrease(params.getDouble("mi"));
    abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
    NoSearch search(problem);
    solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    break;
  }
  case 11: {
    LOG(2) << "running schedule " << schedule << " fast forward portfolio";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(100);
    solver.setSparsification(0);
    solver.setContraction(0);
    FastForward ff(problem);
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    solved = solver.findPlan(ff, search, plan, sameMakespanCount);
    if (!solved) {
      TrueExist abstraction(problem, true);
      solver.setSearchTimeout(0.2);
      solver.setAbstractionTimeout(-1);
      while (!solved) {
        if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
          abstraction.addAllInterferances();
        }
        solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
      }
    }
    break;
  }
  case 12: {
    LOG(2) << "running schedule " << schedule << " greedy best first search";
    solver.setAbstractionTimeout(0);
    solver.setSearchTimeout(-1);
    NoAbstraction abstraction(problem);
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    break;
  }
  case 13: {
    LOG(2) << "running schedule " << schedule << " depth first search";
    solver.setAbstractionTimeout(0);
    solver.setSearchTimeout(-1);
    NoAbstraction abstraction(problem);
    DepthFirstSearch search(problem);
    solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    break;
  }
  case 14: {
    LOG(2) << "running schedule " << schedule << " restart search";
    solver.setAbstractionTimeout(0);
    solver.setSearchTimeout(0.2);
    NoAbstraction abstraction(problem);
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    while (!solved) {
      solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
    }
    break;
  }
  }
  return solved;
}

int main(int argc, char *argv[]) {
  ParameterProcessor params(argc, argv);
  addDefaults(params);
  if (params.getFilename() == "") {
    std::cout << "USAGE: ./pasar_<solver> <planning.sas>\n" << std::endl;
    params.printDefaults();
    return 0;
  }
  int verbosity = params.getInt("v");
  Logger::initLogger(verbosity);
  Logger::logHostname();
  LOG(1) << params;

  std::string problemFile = params.getFilename("sas");
  Problem problem(problemFile);
  LOG(3) << "Variables " << problem.numVariables << " Actions "
         << problem.numActions;
  std::vector<action_t> plan;

  Pasar solver(problem);
  solver.setSparsification(params.getInt("spar"));
  solver.setContraction(params.getInt("cont"));

  const int schedule = params.getInt("s");
  bool solved        = runSchedule(params, solver, problem, schedule, plan);

  if (!solved) {
    LOG(1) << "Problem not solved";
    return 1;
  }

  auto [numRemoved, sumSkipped] = problem.removeLearnedActions(plan);
  LOG(3) << "Used " << numRemoved << " learned actions skipping " << sumSkipped;

  bool valid = validatePlan<false>(plan, problem);
  if (!valid) {
    LOG(1) << "Invalid";
    return 1;
  } else {
    LOG(1) << "Valid plan of length " << plan.size();
  }
  assert(valid);

  std::string outputFile = params.getString("planfile");
  if (!outputFile.empty()) {
    outputPlan(outputFile, problemFile, plan);
  }
  return 0;
}
