#include "flightlib/controller/filter.hpp"

namespace flightlib {

template<typename T>
void filter_pt1<T>::init() {
  u0 = 0;
  y0 = 0;
  y1 = 0;
}

template<>
void filter_pt1<Vector<3>>::init() {
  u0.setZero();
  y0.setZero();
  y1.setZero();
}

template<typename T>
void filter_biquad<T>::init() {
  u0 = 0;
  u1 = 0;
  u2 = 0;
  y0 = 0;
  y1 = 0;
  y2 = 0;
}

template<>
void filter_biquad<Vector<3>>::init() {
  u0.setZero();
  u1.setZero();
  u2.setZero();
  y0.setZero();
  y1.setZero();
  y2.setZero();
}


template<typename T>
void filter_movingAverage<T>::init() {
  for (size_t i = 0; i < length; ++i) {
    x[i] = 0;
  }
}


template<>
void filter_movingAverage<Vector<3>>::init() {
  for (size_t i = 0; i < length; ++i) {
    x[i].setZero();
  }
}

template<typename T>
T filter_movingAverage<T>::update(const T input) {
  std::rotate(x.rbegin(), x.rbegin() + 1, x.rend());
  x[0] = input;
  T sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += x[i];
  }
  return sum / length;
}


template<>
Vector<3> filter_movingAverage<Vector<3>>::update(const Vector<3> input) {
  std::rotate(x.rbegin(), x.rbegin() + 1, x.rend());
  x[0] = input;
  Vector<3> sum;
  sum.setZero();
  for (size_t i = 0; i < length; ++i) {
    sum += x[i];
  }
  return sum / length;
}

}  // namespace flightlib
