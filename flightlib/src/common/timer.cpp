#include "flightlib/common/timer.hpp"

#include <cmath>
#include <limits>

namespace flightlib {

Timer::Timer(const std::string name, const std::string module)
  : name_(name),
    module_(module),
    timing_mean_(0.0),
    timing_last_(0.0),
    timing_S_(0.0),
    timing_min_(std::numeric_limits<Scalar>::max()),
    timing_max_(0.0),
    n_samples_(0) {}

Timer::Timer(const Timer& other)
  : name_(other.name_),
    module_(other.module_),
    t_start_(other.t_start_),
    timing_mean_(other.timing_mean_),
    timing_last_(other.timing_last_),
    timing_S_(other.timing_S_),
    timing_min_(other.timing_min_),
    timing_max_(other.timing_max_),
    n_samples_(other.n_samples_) {}

void Timer::tic() { t_start_ = std::chrono::high_resolution_clock::now(); }

Scalar Timer::toc() {
  // Calculate timing.
  const TimePoint t_end = std::chrono::high_resolution_clock::now();
  timing_last_ = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                          t_end - t_start_)
                          .count();
  n_samples_++;

  // Set timing, filter if already initialized.
  if (timing_mean_ <= 0.0) {
    timing_mean_ = timing_last_;
  } else {
    const Scalar timing_mean_prev = timing_mean_;
    timing_mean_ =
      timing_mean_prev + (timing_last_ - timing_mean_prev) / n_samples_;
    timing_S_ = timing_S_ + (timing_last_ - timing_mean_prev) *
                              (timing_last_ - timing_mean_);
  }
  timing_min_ = (timing_last_ < timing_min_) ? timing_last_ : timing_min_;
  timing_max_ = (timing_last_ > timing_max_) ? timing_last_ : timing_max_;

  t_start_ = t_end;

  return timing_mean_;
}

Scalar Timer::operator()() const { return timing_mean_; }

Scalar Timer::mean() const { return timing_mean_; }

Scalar Timer::last() const { return timing_last_; }

Scalar Timer::min() const { return timing_min_; }

Scalar Timer::max() const { return timing_max_; }

Scalar Timer::std() const { return std::sqrt(timing_S_ / n_samples_); }

int Timer::count() const { return n_samples_; }

void Timer::reset() {
  n_samples_ = 0u;
  t_start_ = TimePoint();
  timing_mean_ = 0.0;
  timing_last_ = 0.0;
  timing_S_ = 0.0;
  timing_min_ = std::numeric_limits<Scalar>::max();
  timing_max_ = 0.0;
}

void Timer::print() const { std::cout << *this; }

std::ostream& operator<<(std::ostream& os, const Timer& timer) {
  if (!timer.module_.empty()) os << "[" << timer.module_ << "] ";

  if (timer.n_samples_ < 1) {
    os << "Timing " << timer.name_ << " has no call yet." << std::endl;
    return os;
  }

  const std::streamsize prec = os.precision();
  os.precision(3);

  os << "Timing " << timer.name_ << " in " << timer.n_samples_ << " calls"
     << std::endl;

  if (!timer.module_.empty()) os << "[" << timer.module_ << "] ";
  os << "mean|std:  " << 1000 * timer.timing_mean_ << " | "
     << 1000 * timer.timing_S_ << " ms    "
     << "[min|max:  " << 1000 * timer.timing_min_ << " | "
     << 1000 * timer.timing_max_ << " ms]" << std::endl;

  os.precision(prec);
  return os;
}

ScopedTimer::ScopedTimer(const std::string name, const std::string module)
  : Timer(name, module) {
  this->tic();
}

ScopedTimer::~ScopedTimer() {
  this->toc();
  this->print();
}

}  // namespace flightlib
