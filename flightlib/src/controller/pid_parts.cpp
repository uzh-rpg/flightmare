#include "flightlib/controller/pid_parts.hpp"


namespace flightlib {

pidD::pidD(const float fs)
  : fs(fs),
    gyro_lpf_1(f_gyro_lpf_1, fs),
    gyro_lpf_2(f_gyro_lpf_2, fs),
    dterm_gyro_lpf(f_dterm_lpf, fs) {
  last_gyro.setZero();
}

Vector<3> pidD::update(const Vector<3>& bodyRate) {
  const Vector<3> filtered_gyro =
    gyro_lpf_2.update(gyro_lpf_1.update(bodyRate));
  const Vector<3> d_part = dterm_gyro_lpf.update(filtered_gyro - last_gyro);
  last_gyro = filtered_gyro;

  return (d_part.array() * d_gain.array()) * fs / 1e3;
}


pidI::pidI(const float fs) : fs(fs) {}

Vector<3> pidI::update(const Vector<3>& setpoint, const Vector<3>& bodyRate) {
  i_part += (setpoint - bodyRate) * 1e3 / fs;
  i_part = i_part.cwiseMax(-limit).cwiseMin(limit);
  return (i_part.array() * i_gain.array()).matrix();
}


}  // namespace flightlib
