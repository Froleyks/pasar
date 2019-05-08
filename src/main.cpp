#include <iostream>

#include "problem/problem.hpp"
#include "tools/logger.hpp"
#include "tools/parameter_processor.hpp"

int main(int argc, char *argv[]) {
  ParameterProcessor params(argc, argv);
  if (params.getFilename() == "") {
    std::cout << "USAGE: ./ADAL_<solver> <planning.sas>\n" << std::endl;
    params.printDefaults();
    return 1;
  }
  int verbosity = params.getInt("v");
  Logger::initLogger(verbosity);

  Logger::logHostname();

  Problem problem(params.getFilename());
  return 0;
}
