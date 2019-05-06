#include <iostream>

#include "tools/logger.hpp"
#include "tools/parameter_processor.hpp"

int main(int argc, char *argv[]) {
  ParameterProcessor params(argc, argv);
  if (params.getFilename() == "") {
    std::cout << "USAGE: ./ADAL_<solver> <planning.sas>\n" << std::endl;
    params.printDefaults();
    return 1;
  }
  Logger::logHostname();
  int verbosity = params.getInt("v");
  Logger::setLogLevel(verbosity);
  return 0;
}
