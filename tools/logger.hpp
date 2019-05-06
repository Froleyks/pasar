#pragma once

#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#define log(level)                                                             \
  if (level > Logger::LOG_LEVEL)                                               \
    ;                                                                          \
  else                                                                         \
    Logger::getLogger().get(level)

namespace Logger {
int LOG_LEVEL = 0;

void setLogLevel(int logLevel) { LOG_LEVEL = logLevel; }

double getAbsoluteTime() {
  timeval time;
  gettimeofday(&time, NULL);
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

static double start = getAbsoluteTime();

double getTime() { return getAbsoluteTime() - start; }

void setVerbosityLevel(int level) { LOG_LEVEL = level; }

class Log {
private:
  std::ostringstream os;

public:
  Log() {}
  ~Log();
  std::ostringstream &get(int level);
};

Log::~Log() { std::cout << os.str() << std::endl; }

std::ostringstream &Log::get(int level) {
  os << std::setw(10) << std::fixed << std::setprecision(6) << getTime() << " "
     << level << ": ";
  return os;
}

Log getLogger() { return Log(); }

void logHostname() {
  char hostname[HOST_NAME_MAX];
  gethostname(hostname, HOST_NAME_MAX);
  log(1) << "hostname: " << hostname;
}

bool tableOpen           = true;
unsigned numberOfColumns = 0;
std::string table        = "|";
std::vector<int> columnSums;

unsigned currentColumn = 0;

void addColumn(std::string column) {
  numberOfColumns++;
  columnSums.push_back(0);
  std::ostringstream os;
  os << std::setw(9) << column << " |";
  table += os.str();
}

void logTable() {
  for (unsigned i = 0; i < numberOfColumns; ++i) {
    table += "----------+";
  }
  table.back() = '|';
  table += "\n|";
  std::ostringstream os;
  for (auto s : columnSums) {
    os << std::setw(9) << s << " |";
  }
  table += os.str();

  std::cout << table << std::endl;
}

static void interrupted(int x) {
  std::cout << table << std::endl;
  std::cout << " Interrupted" << getTime() << " " << x;
  std::abort();
}

template <class T> void insertTable(T value) {
  if (tableOpen) {
    tableOpen = false;
    table += "\n|";
    for (unsigned i = 0; i < numberOfColumns; ++i) {
      table += "----------+";
    }
    table.back() = '|';
    table += "\n|";

    signal(SIGINT, Logger::interrupted);
    signal(SIGTERM, Logger::interrupted);
  }
  std::ostringstream os;
  os << std::setw(9) << value << " |";
  columnSums[currentColumn] += value;
  currentColumn++;
  table += os.str();
  if (currentColumn == numberOfColumns) {
    table += "\n|";
    currentColumn = 0;
  }
}

} // namespace Logger
