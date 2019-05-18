#pragma once

#include <iomanip>
#include <iostream>
#include <limits.h>
#include <sys/time.h>
#include <unistd.h>

#define log(level)                                                             \
  if (level > Logger::currentLogLevel())                                       \
    ;                                                                          \
  else                                                                         \
    Logger::getLogger().get(level)

namespace Logger {

inline int &currentLogLevel() {
  static int LOG_LEVEL;
  return LOG_LEVEL;
}

inline double &start() {
  static double startTime;
  return startTime;
}

inline double getAbsoluteTime() {
  timeval time;
  gettimeofday(&time, NULL);
  return static_cast<double>(time.tv_sec) +
         static_cast<double>(time.tv_usec) * .000001;
}

inline void initLogger(int logLevel) {
  start()           = getAbsoluteTime();
  currentLogLevel() = logLevel;
}

inline double getTime() { return getAbsoluteTime() - start(); }

class Log {
private:
  std::ostringstream os{};

public:
  Log() {}
  ~Log();
  std::ostringstream &get(int level);
};

inline Log::~Log() { std::cout << os.str() << std::endl; }

inline std::ostringstream &Log::get(int level) {
  os << std::setw(10) << std::fixed << std::setprecision(6) << getTime() << " "
     << level << ": ";
  return os;
}

inline Log getLogger() { return Log(); }

inline void logHostname() {
  char hostname[HOST_NAME_MAX];
  gethostname(hostname, HOST_NAME_MAX);
  log(1) << "hostname: " << hostname;
}

} // namespace Logger
