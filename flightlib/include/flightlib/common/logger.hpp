// """Credit to Philipp Foehn"""
#pragma once

#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "flightlib/common/types.hpp"

namespace flightlib {


class Logger {
 public:
  Logger(const std::string& name, const bool color = true);
  Logger(const std::string& name, const std::string& filename);
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  inline std::streamsize precision(const std::streamsize n);
  inline void scientific(const bool on = true);

  void info(const char* msg, ...) const;
  void warn(const char* msg, ...) const;
  void error(const char* msg, ...) const;
  void fatal(const char* msg, ...) const;

  template<typename T>
  std::ostream& operator<<(const T& printable) const;

  inline const std::string& name() const { return name_; }

  static constexpr int MAX_CHARS = 256;

 private:
  static constexpr int DEFAULT_PRECISION = 3;
  static constexpr int NAME_PADDING = 15;
  static constexpr char RESET[] = "\033[0m";
  static constexpr char RED[] = "\033[31m";
  static constexpr char YELLOW[] = "\033[33m";
  static constexpr char INFO[] = "Info:    ";
  static constexpr char WARN[] = "Warning: ";
  static constexpr char ERROR[] = "Error:   ";
  static constexpr char FATAL[] = "Fatal:   ";

  std::string name_;
  mutable std::ostream sink_;
  const bool colored_;
};

template<typename T>
std::ostream& Logger::operator<<(const T& printable) const {
  return sink_ << name_ << printable;
}

}  // namespace flightlib 
