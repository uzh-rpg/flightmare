#pragma once

#include <algorithm>
#include <vector>

#include "flightlib/common/types.hpp"
#include "flightlib/controller/filter.hpp"

namespace flightlib {
class pidD {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  pidD(const float fs);
  Vector<3> update(const Vector<3>&);

 private:
  const float f_gyro_lpf_1 = 350;  // Hz
  const float f_gyro_lpf_2 = 250;  // Hz
  const float f_dterm_lpf = 170;   // Hz
  const float fs;
  const Vector<3> d_gain = (Vector<3>() << -625.253, -630.742, 0).finished();

  filter_pt1<Vector<3>> gyro_lpf_1;
  filter_pt1<Vector<3>> gyro_lpf_2;
  filter_pt1<Vector<3>> dterm_gyro_lpf;
  Vector<3> last_gyro;
};


class pidI {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  pidI(const float fs);
  Vector<3> update(const Vector<3>&, const Vector<3>&);

 private:
  const float fs;
  const Vector<3> i_gain = (Vector<3>() << 1.0, 1.0, 1.394).finished();
  const float limit = 100;
  Vector<3> i_part = (Vector<3>() << 0, 0, 0).finished();
};


class pidP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector<3> update(const Vector<3>& setpoint, const Vector<3>& bodyRate) {
    const Vector<3> error = setpoint - bodyRate;
    return (p_gain.array() * error.array()).matrix();
  }

 private:
  const Vector<3> p_gain = (Vector<3>() << 72.706, 72.892, 49.385).finished();
};

}  // namespace flightlib
