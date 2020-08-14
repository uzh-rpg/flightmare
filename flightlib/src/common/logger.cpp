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
      warn("Could not open file %s. Logging to console!", filename);
  }
  sink_.precision(DEFAULT_PRECISION);
}

Logger::~Logger() {}

inline std::streamsize Logger::precision(const std::streamsize n) {
  return sink_.precision(n);
}

inline void Logger::scientific(const bool on) {
  if (on)
    sink_ << std::scientific;
  else
    sink_ << std::fixed;
}

void Logger::info(const std::string& message) const {
  if (colored_)
    sink_ << name_ << message << std::endl;
  else
    sink_ << name_ << INFO << message << std::endl;
}

void Logger::warn(const std::string& message) const {
  if (colored_)
    sink_ << YELLOW << name_ << message << RESET << std::endl;
  else
    sink_ << name_ << WARN << message << std::endl;
}

void Logger::error(const std::string& message) const {
  if (colored_)
    sink_ << RED << name_ << message << RESET << std::endl;
  else
    sink_ << name_ << ERROR << message << std::endl;
}

}  // namespace flightlib