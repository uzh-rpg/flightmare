#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "flightlib/common/types.hpp"

namespace flightlib {

// Filter design from https://www.ti.com/lit/an/slaa447/slaa447.pdf
template<typename T>
struct filter_biquad {
  const float Q = 1 / sqrt(2);
  const float fc, fs;

  // omega = 2* pi * f_corner / fs
  const float omega;
  const float cs, sn, alpha;
  float a0, a1, a2, b0, b1, b2;
  T u0, u1, u2;
  T y0, y1, y2;

  filter_biquad(const float fc, const float fs)
    : fc(fc),
      fs(fs),
      omega(2 * M_PI * fc / fs),
      cs(std::cos(omega)),
      sn(std::sin(omega)),
      alpha(sn / (2.0f * Q)),
      a0(1 + alpha),
      a1((-2 * cs) / a0),
      a2((1 - alpha) / a0),
      b0(((1 - cs) * 0.5) / a0),
      b1((1 - cs) / a0),
      b2(((1 - cs) * 0.5f) / a0) {
    init();
  }

  T update(const T input) {
    u0 = input;
    y0 = b0 * u0 + b1 * u1 + b2 * u2 - a1 * y1 - a2 * y2;
    u2 = u1;
    u1 = u0;
    y2 = y1;
    y1 = y0;
    return y0;
  }

 private:
  void init();
};


// Filter design from
// https://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
template<typename T>
struct filter_pt1 {
  const float fc;
  const float fs;
  T u0, y0, y1;

  // omega = 2* pi * f_corner / fs
  const float omega;
  const float b0, a1;

  filter_pt1(const float fc, const float fs)
    : fc(fc),
      fs(fs),
      omega(2 * M_PI * fc / fs),
      b0(omega / (omega + 1)),
      a1(b0 - 1) {
    init();
  };

  T update(const T input) {
    u0 = input;
    y0 = u0 * b0 - y1 * a1;
    y1 = y0;
    return y0;
  }

 private:
  void init();
};

// Own filter design
template<typename T>
struct filter_movingAverage {
  const size_t length;
  std::vector<T> x;

  filter_movingAverage(const size_t length) : length(length), x(length) {
    init();
  }
  T update(const T input);

 private:
  void init();
};

}  // namespace flightlib
