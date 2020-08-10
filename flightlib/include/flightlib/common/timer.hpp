#pragma once

#include <chrono>
#include <iostream>
#include <string>

#include "flightlib/common/types.hpp"

namespace flightlib {

/*
 * Timer class to perform runtime analytics.
 *
 * This timer class provides a simple solution to time code.
 * Simply construct a timer and call it's `tic()` and `toc()` functions to time
 * code. It is intended to be used to time multiple calls of a function and not
 * only reports the `last()` timing, but also statistics such as the `mean()`,
 * `min()`, `max()` time, the `count()` of calls to the timer , and even
 * standard deviation `std()`.
 *
 * The constructor can take a name for the timer (like "update") and a name for
 * the module (like "Filter").
 * After construction it can be `reset()` if needed.
 *
 * A simple way to get the timing and stats is `std::cout << timer;` which can
 * output to arbitrary streams, overloading the stream operator,
 * or `print()` which always prints to console.
 *
 */
class Timer {
 public:
  Timer(const std::string name = "", const std::string module = "");
  Timer(const Timer& other);
  ~Timer() {}

  /// Start the timer.
  void tic();

  /// Stops timer, calculates timing, also tics again.
  Scalar toc();

  /// Reset saved timings and calls;
  void reset();

  // Accessors
  Scalar operator()() const;
  Scalar mean() const;
  Scalar last() const;
  Scalar min() const;
  Scalar max() const;
  Scalar std() const;
  int count() const;

  /// Custom stream operator for outputs.
  friend std::ostream& operator<<(std::ostream& os, const Timer& timer);

  /// Print timing information to console.
  void print() const;

 private:
  std::string name_, module_;
  using TimePoint = std::chrono::high_resolution_clock::time_point;
  TimePoint t_start_;

  // Initialize timing to impossible values.
  Scalar timing_mean_;
  Scalar timing_last_;
  Scalar timing_S_;
  Scalar timing_min_;
  Scalar timing_max_;

  int n_samples_;
};

/*
 * Simple class to time scopes.
 *
 * This effectively instantiates a timer and calls `tic()` in its constructor
 * and `toc()` and ` print()` in its destructor.
 */
class ScopedTimer : public Timer {
 public:
  ScopedTimer(const std::string name = "", const std::string module = "");
  ~ScopedTimer();
};

}  // namespace flightlib