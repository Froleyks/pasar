#include <iostream>

#include "tools/logger.hpp"
#include "tools/parameter_processor.hpp"

#include "src/pasar.hpp"
#include "src/problem/problem.hpp"
#include "src/problem/validate_plan.hpp"

#include "abstraction/cycle_break.hpp"
// #include "abstraction/fast_forward.hpp"
// #include "abstraction/leaf_reduction.hpp"
#include "abstraction/exist.hpp"
#include "abstraction/foreach.hpp"
#include "abstraction/no_abstraction.hpp"

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
                    "abstraction and search schedules\n"
                    "\t 0: pasar\n"
                    "\t 1: portfolio\n"
                    "\t 2: encoding foreach\n"
                    "\t 3: encoding exists\n"
                    "\t 4: encoding cycle break\n"
                    "\t 5: depth first search");
  params.addDefault("spar", "2",
                    "sparsification\n"
                    "\t 0: complete states\n"
                    "\t 1: partial states\n"
                    "\t 2: action elimination");
  params.addDefault(
      "cont", "0",
      "contraction\n"
      "\t 1: additional actions are learned and fewer guide states are used");
  params.addDefault("i", "0",
                    "interleave\n"
                    "\t 1: interleave search and heuristic computation");

  // abstraction
  params.addDefault("a", "0",
                    "abstraction\n"
                    "\t 0: foreach\n"
                    "\t 1: exist\n"
                    "\t 2: cycle break");
  params.addDefault("at", "-1",
                    "timeout for abstraction phase[s]\n"
                    "\t-1: infinity\n"
                    "\t 0: disable abstraction");
  params.addDefault("ml", "20000",
                    "\tnumber of conflicts(CDCL) before makespan is aborted\n"
                    "\t-1: infinity\n"
                    "\t 0: disable abstraction");
  params.addDefault("mt", "-1",
                    "timeout for per makespan[s]\n"
                    "\t-1: infinity");
  params.addDefault("im", "5", "initial makespan");
  params.addDefault("mi", "1.2", "makespan increase");
  params.addDefault(
      "sml", "40",
      "same makespan limit\n"
      "\t if enough abstractions are solved in the same makespan\n"
      "\t the cegar approach is aborted\n"
      "\t 0: no limit");
  params.addDefault("ef", "1",
                    "fallback for exist if sml is reached\n"
                    "\t 0: exist\n"
                    "\t 1: foreach\n"
                    "\t 2: cycle break");

  // search
  params.addDefault("sl", "50000",
                    "\tnumber of nodes explored before search gives up\n"
                    "\t-1: infinity\n"
                    "\t 0: disable search");
  params.addDefault("st", "-1",
                    "timeout for search phase[s]\n"
                    "\t-1: infinity\n"
                    "\t 0: disable search");
  params.addDefault("seed", "42",
                    "\t seed used for random tie breaking during search");
  params.addDefault(
      "gdf", "1.2",
      "gainDecayFactor ≥ 1\n"
      "\t high gainDecayFactor ⇒ guideStates further from the goal\n"
      "\t have less impact on the direction of the search");
}

template <class Abstraction>
void runEncoding(const ParameterProcessor &params, Abstraction &abstraction,
                 std::vector<action_t> &plan) {
  static_assert(Abstraction::isSatBased);
  abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
  abstraction.setMakespanIncrease(params.getDouble("mi"));
  abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));
  std::vector<AbstractPlan::Step> steps;
  abstraction.solve(steps);
  // flatten plan
  plan.reserve(steps.size());
  for (auto &s : steps) {
    std::vector<action_t> planForStep ;
    abstraction.fixStep(s, planForStep);
    plan.insert(plan.end(), planForStep.begin(), planForStep.end());
  }
}

template <class Abstraction, class Search>
void runPasar(const ParameterProcessor &params, Pasar &solver,
              Abstraction &abstraction, Search &search,
              std::vector<action_t> &plan) {
  unsigned sameMakespanLimit = static_cast<unsigned>(params.getInt("sml"));
  unsigned sameMakespanCount = 0;
  solver.setAbstractionTimeout(params.getDouble("at"));
  solver.setSearchTimeout(params.getDouble("st"));

  abstraction.setInitialMakespan(static_cast<unsigned>(params.getInt("im")));
  abstraction.setMakespanIncrease(params.getDouble("mi"));
  abstraction.setTimeLimitPerMakespan(params.getDouble("mt"));

  search.setSeed(static_cast<unsigned>(params.getInt("seed")));
  search.setGainDecayFactor(params.getDouble("gdf"));

  bool solved = false;
  while (!solved) {
    if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
      abstraction.refine();
    }
    solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
  }
}

void runSchedule(const ParameterProcessor &params, Pasar &solver,
                 Problem &problem, int schedule, std::vector<action_t> &plan) {
  switch (schedule) {
  case 0: {
    GreedyBestFirst search(problem);
    switch (params.getInt("a")) {
    case 0: {
      LOG(2) << "running schedule " << schedule << " pasar " << "foreach";
      Foreach abstraction(problem, true);
      runPasar(params, solver, abstraction, search, plan);
      break;
    }
    case 1: {
      LOG(2) << "running schedule " << schedule << " pasar " << "exist";
      Exist abstraction(problem, true);
      abstraction.setFallback(params.getInt("ef"));
      runPasar(params, solver, abstraction, search, plan);
      break;
    }
    case 2: {
      LOG(2) << "running schedule " << schedule << " pasar " << "cycle break";
      CycleBreak abstraction(problem, true);
      runPasar(params, solver, abstraction, search, plan);
      break;
    }
    }
    break;
  }
  case 1: {
    LOG(2) << "running schedule " << schedule << " portfolio";
    solver.setAbstractionTimeout(0);
    solver.setSearchTimeout(100);
    NoAbstraction dummy(problem);
    GreedyBestFirst search(problem);
    search.setSeed(static_cast<unsigned>(params.getInt("seed")));
    search.setGainDecayFactor(params.getDouble("gdf"));
    unsigned sameMakespanLimit = static_cast<unsigned>(params.getInt("sml"));
    unsigned sameMakespanCount = 0;
    bool solved                = false;
    solved = solver.findPlan(dummy, search, plan, sameMakespanCount);
    if (!solved) {
      Exist abstraction(problem, true);
      abstraction.setFallback(params.getInt("ef"));
      solver.setSearchTimeout(params.getDouble("st"));
      solver.setAbstractionTimeout(params.getDouble("at"));
      while (!solved) {
        if (sameMakespanLimit && sameMakespanCount >= sameMakespanLimit) {
          abstraction.refine();
        }
        solved = solver.findPlan(abstraction, search, plan, sameMakespanCount);
      }
    }
    break;
  }
  case 2: {
    LOG(2) << "running schedule " << schedule << " encoding foreach";
    Foreach abstraction(problem);
    runEncoding(params, abstraction, plan);
    break;
  }
  case 3: {
    LOG(2) << "running schedule " << schedule << " encoding true exist";
    Exist abstraction(problem);
    runEncoding(params, abstraction, plan);
    break;
  }
  case 4: {
    LOG(2) << "running schedule " << schedule << " encoding cycle break";
    CycleBreak abstraction(problem);
    runEncoding(params, abstraction, plan);
    break;
  }
  case 5: {
    LOG(2) << "running schedule " << schedule << " depth first search";
    DepthFirstSearch search(problem);
    std::vector<State> guideStates{problem.goalState};
    search.search(guideStates, plan);
    break;
  }
  }
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
  size_t seed             = static_cast<size_t>(params.getInt("seed"));
  Problem problem(problemFile, seed);
  LOG(3) << "Variables " << problem.numVariables << " Actions "
         << problem.numActions;
  std::vector<action_t> plan;

  Pasar solver(problem);
  solver.setSparsification(params.getInt("spar"));
  solver.setContraction(params.getInt("cont"));

  const int schedule = params.getInt("s");
  runSchedule(params, solver, problem, schedule, plan);

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
