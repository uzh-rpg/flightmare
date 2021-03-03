#include "flightlib/common/logger.hpp"

namespace flightlib {

Logger::Logger(const std::string& name, const bool color)
  : sink_(std::cout.rdbuf()), colored_(color) {
  name_ = "[" + name + "]";

  if (name_.size() < NAME_PADDING)
    name_ = name_ + std::string(NAME_PADDING - name_.size(), ' ');
  else
    name_ = name_ + " ";

  sink_.precision(DEFAULT_PRECISION);
}

Logger::Logger(const std::string& name, const std::string& filename)
  : Logger(name, false) {
  if (!filename.empty()) {
    std::filebuf* fbuf = new std::filebuf;
    if (fbuf->open(filename, std::ios::out))
      sink_.rdbuf(fbuf);
    else
      warn("Could not open file %s. Logging to console!", filename.c_str());
  }
  sink_.precision(DEFAULT_PRECISION);
}

inline std::streamsize Logger::precision(const std::streamsize n) {
  return sink_.precision(n);
}

inline void Logger::scientific(const bool on) {
  if (on)
    sink_ << std::scientific;
  else
    sink_ << std::fixed;
}

void Logger::info(const char* msg, ...) const {
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS)
    sink_ << name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    sink_ << name_ << buf << std::endl;
  else
    sink_ << name_ << INFO << buf << std::endl;
}

void Logger::warn(const char* msg, ...) const {
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS)
    sink_ << name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    sink_ << YELLOW << name_ << buf << RESET << std::endl;
  else
    sink_ << name_ << WARN << buf << std::endl;
}

void Logger::error(const char* msg, ...) const {
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS)
    sink_ << name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    sink_ << RED << name_ << buf << RESET << std::endl;
  else
    sink_ << name_ << ERROR << buf << std::endl;
}

void Logger::fatal(const char* msg, ...) const {
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS)
    sink_ << name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    sink_ << RED << name_ << buf << RESET << std::endl;
  else
    sink_ << name_ << FATAL << buf << std::endl;
  throw std::runtime_error(name_ + buf);
}
}  // namespace flightlib
