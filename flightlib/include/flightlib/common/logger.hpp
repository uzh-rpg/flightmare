// """credit: Philipp Foehn """
#pragma once

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

namespace flightlib {

class Logger {
 public:
  Logger(const std::string& name, const bool color = true);
  Logger(const std::string& name, const std::string& filename);
  ~Logger();

  inline std::streamsize precision(const std::streamsize n);
  inline void scientific(const bool on = true);

  template<class... Args>
  void info(const std::string& message, const Args&... args) const;
  void info(const std::string& message) const;

  template<class... Args>
  void warn(const std::string& message, const Args&... args) const;
  void warn(const std::string& message) const;

  template<class... Args>
  void error(const std::string& message, const Args&... args) const;
  void error(const std::string& message) const;

  template<class... Args>
  void fatal(const std::string& message, const Args&... args) const;
  void fatal(const std::string& message) const;

  template<typename T>
  std::ostream& operator<<(const T& printable) const;

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
  //
  std::string name_;
  mutable std::ostream sink_;
  const bool colored_;
};

template<class... Args>
void Logger::info(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    info(buf);
  else
    error("=== Logging error ===\n");
}

template<class... Args>
void Logger::warn(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    warn(buf);
  else
    error("=== Logging error ===\n");
}

template<class... Args>
void Logger::error(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    error(buf);
  else
    error("=== Logging error ===\n");
}

template<class... Args>
void Logger::fatal(const std::string& message, const Args&... args) const {
  char buf[MAX_CHARS];
  const int n = std::snprintf(buf, MAX_CHARS, message.c_str(), args...);
  if (n >= 0 && n < MAX_CHARS)
    fatal(buf);
  else
    fatal("=== Logging error ===\n");
}

template<typename T>
std::ostream& Logger::operator<<(const T& printable) const {
  return sink_ << name_ << printable;
}

}  // namespace flightlib