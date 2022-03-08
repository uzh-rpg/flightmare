#pragma once

#include "flightlib/common/types.hpp"

namespace flightlib {

namespace quadcmd {

enum CMDMODE : int {
  SINGLEROTOR = 0,
  THRUSTRATE = 1,
};

}  // namespace quadcmd
class Command {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Command();
  ~Command();

  //
  bool valid() const;
  bool isSingleRotorThrusts() const;
  bool isThrustRates() const;

  //
  void setZeros(void);
  void setCmdVector(const Vector<4>& cmd);
  bool setCmdMode(const int cmd_mode);

  /// time in [s]
  Scalar t;

  /// Single rotor thrusts in [N]
  Vector<4> thrusts;

  /// Collective mass-normalized thrust in [m/s^2]
  Scalar collective_thrust;

  /// Bodyrates in [rad/s]
  Vector<3> omega;

  ///
  int cmd_mode;
};

}  // namespace flightlib