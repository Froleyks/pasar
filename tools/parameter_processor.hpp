#pragma once

#include <iomanip>
#include <iostream>
#include <map>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

class ParameterProcessor {
private:
  std::map<std::string, std::string> filenames;
  std::map<std::string, std::string> parameters;
  std::string lastAddedFilename = "";

  struct Default {
    std::string name_;
    std::string defaultValue_;
    std::string description_;
    Default(std::string name, std::string defaultValue, std::string description)
        : name_(name), defaultValue_(defaultValue), description_(description) {}
  };

  std::vector<Default> defaultParameters;

public:
  ParameterProcessor() : filenames(), parameters(), defaultParameters() {}

  ParameterProcessor(int argc, char **argv) : filenames(), parameters(), defaultParameters() { init(argc, argv); }

  void init(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
      char *arg = argv[i];
      if (arg[0] == '-') {
        // parameter
        arg++;
        if (i + 1 == argc || argv[i + 1][0] == '-') {
          parameters[arg] = "1";
        } else {
          i++;
          parameters[arg] = argv[i];
        }
      } else {
        // filename
        std::string filename(arg);
        std::string extension;
        char *p;
        strtok(arg, ".");
        p = strtok(NULL, ".");
        while (p) {
          extension = std::string(p);
          p         = strtok(NULL, ".");
        }
        filenames[extension] = filename;
        lastAddedFilename    = filename;
      }
    }
  }

  bool isSet(std::string name) {
    return parameters.find(name) != parameters.end();
  }

  int getInt(std::string name, int defaultValue = 0) {
    auto val = parameters.find(name);
    if (val == parameters.end()) {
      return defaultValue;
    }
    return stoi(val->second);
  }

  bool set(std::string name, std::string value) {
    if (isSet(name)) {
      return false;
    }
    parameters[name] = value;
    return true;
  }

  double getDouble(std::string name, double defaultValue = 0) {
    auto val = parameters.find(name);
    if (val == parameters.end()) {
      return defaultValue;
    }
    return stod(val->second);
  }

  std::string getString(std::string name, std::string defaultValue = "") {
    auto val = parameters.find(name);
    if (val == parameters.end()) {
      return defaultValue;
    }
    return val->second;
  }

  std::string getFilename(std::string extension = "UNDEFINED") {
    auto val = filenames.find(extension);
    if (val == filenames.end()) {
      return lastAddedFilename;
    }
    return val->second;
  }

  friend std::ostream &operator<<(std::ostream &stream,
                                  const ParameterProcessor &params) {
    stream << "Filenames:" << std::endl;
    for (const auto &filename : params.filenames) {
      stream << filename.second << std::endl;
    }
    stream << std::endl;
    stream << "Parameters:" << std::endl;
    for (const auto &parameter : params.parameters) {
      stream << parameter.first << " = " << parameter.second << std::endl;
    }
    return stream;
  }

  void addDefault(std::string name, std::string defaultValue,
                  std::string description) {
    if (parameters.find(name) == parameters.end()) {
      parameters[name] = defaultValue;
    }
    defaultParameters.emplace_back(name, defaultValue, description);
  }

  void printDefaults() {
    std::cout << "###Default Parameters###" << std::endl;
    size_t maxName        = 0;
    size_t maxValue       = 0;
    size_t maxDescription = 0;
    for (auto [name, defaultValue, description] : defaultParameters) {
      if (name.length() > maxName) {
        maxName = name.length();
      }
      if (defaultValue.length() > maxValue) {
        maxValue = defaultValue.length();
      }
      if (description.length() > maxDescription) {
        maxDescription = description.length();
      }
    }
    maxName++;
    maxValue++;
    for (auto [name, defaultValue, description] : defaultParameters) {
      std::cout << std::right << std::setw(static_cast<int>(maxName)) << name;
      std::cout << std::right << std::setw(static_cast<int>(maxValue))
                << defaultValue << "  ";
      std::cout << std::left << description;
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
};
