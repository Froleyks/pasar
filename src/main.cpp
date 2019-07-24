#include <iostream>

#include "tools/logger.hpp"
#include "tools/parameter_processor.hpp"

#include "src/pasar.hpp"
#include "src/problem/problem.hpp"
#include "src/problem/validate_plan.hpp"

#include "abstraction/foreach.hpp"
#include "abstraction/no_guidance.hpp"
#include "abstraction/true_exist.hpp"

#include "src/search/depth_first_search.hpp"
#include "src/search/greedy_best_first.hpp"

void outputPlan(std::string &outputFile, std::string &problemFile,
                std::vector<action_t> &plan) {
  std::string printPlan = Problem::planToString(problemFile, plan);
  std::ofstream out(outputFile);
  out << printPlan;
  out.close();
}

void addDefaults(ParameterProcessor &params) {
  params.addDefault("s", "1", "used abstraction and search schedule");
  params.addDefault("spar", "2",
                    "sparsification\n"
                    "\t0: complete states\n"
                    "\t1: partial states\n"
                    "\t2: action elimination");
  params.addDefault(
      "cont", "0",
      "1: additional actions are learned and fewer guide states are used");
}

bool runSchedule(Pasar &solver, Problem &problem, int schedule,
                 std::vector<action_t> &plan) {

  bool solved = false;
  switch (schedule) {
  case 0: {
    log(2) << "running schedule " << schedule << " depth first search";
    solver.setAbstractionTimeout(0);
    solver.setSearchTimeout(-1);
    NoGuidance abstraction(problem);
    DepthFirstSearch search(problem);
    solved = solver.findPlan(abstraction, search, plan);
    break;
  }
  case 1: {
    log(2) << "running schedule " << schedule << " greedy best first search";
    solver.setAbstractionTimeout(0);
    solver.setSearchTimeout(-1);
    NoGuidance abstraction(problem);
    GreedyBestFirst search(problem);
    while (!solved) {
      solved = solver.findPlan(abstraction, search, plan);
    }
    break;
  }
  case 2: {
    log(2) << "running schedule " << schedule << " foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    Foreach abstraction(problem);
    DepthFirstSearch search(problem);

    solved = solver.findPlan(abstraction, search, plan);
    break;
  }
  case 3: {
    log(2) << "running schedule " << schedule << " pure cegar foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    Foreach abstraction(problem, true);
    DepthFirstSearch search(problem);
    while (!solved) {
      solved = solver.findPlan(abstraction, search, plan);
    }
    break;
  }
  case 4: {
    log(2) << "running schedule " << schedule << " cegar foreach";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(1);
    Foreach abstraction(problem, true);
    GreedyBestFirst search(problem);
    while (!solved) {
      solved = solver.findPlan(abstraction, search, plan);
    }
    break;
  }
  case 5: {
    log(2) << "running schedule " << schedule << " true exist";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    TrueExist abstraction(problem);
    DepthFirstSearch search(problem);
    while (!solved) {
      solved = solver.findPlan(abstraction, search, plan);
    }
    break;
  }
  case 6: {
    log(2) << "running schedule " << schedule << " pure cegar true exist";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(0);
    TrueExist abstraction(problem, true);
    DepthFirstSearch search(problem);
    while (!solved) {
      solved = solver.findPlan(abstraction, search, plan);
    }
    break;
  }
  case 7: {
    log(2) << "running schedule " << schedule << " cegar true exist";
    solver.setAbstractionTimeout(-1);
    solver.setSearchTimeout(1);
    TrueExist abstraction(problem, true);
    GreedyBestFirst search(problem);
    while (!solved) {
      solved = solver.findPlan(abstraction, search, plan);
    }
    break;
  }
  default:
    log(1) << "Configuration not found";
    return false;
    break;
  }
  return solved;
}

int main(int argc, char *argv[]) {
  ParameterProcessor params(argc, argv);
  addDefaults(params);
  if (params.getFilename() == "") {
    std::cout << "USAGE: ./pasar_<solver> <planning.sas>\n" << std::endl;
    params.printDefaults();
    return 1;
  }
  int verbosity = params.getInt("v");
  Logger::initLogger(verbosity);

  Logger::logHostname();

  log(1) << params;

  std::string problemFile = params.getFilename("sas");

  Problem problem(problemFile);
  log(3) << "Variables " << problem.numVariables << " Actions "
         << problem.numActions;
  std::vector<action_t> plan;

  Pasar solver(problem);
  solver.setSparsification(params.getInt("spar"));
  solver.setContraction(params.getInt("cont"));

  const int schedule = params.getInt("s");
  bool solved        = runSchedule(solver, problem, schedule, plan);

  if (!solved) {
    log(1) << "Problem not solved";
    return 1;
  }

  problem.removeLearnedActions(plan);

  bool valid = validatePlan<false>(plan, problem);
  if (!valid) {
    log(1) << "Invalid";
    return 1;
  } else {
    log(1) << "Valid plan of length " << plan.size();
  }
  assert(valid);

  std::string outputFile = params.getString("planfile");
  if (!outputFile.empty()) {
    outputPlan(outputFile, problemFile, plan);
  }
  return 0;
}
