
#pragma once

#include <cmath>

#include "flightlib/common/types.hpp"

namespace flightlib {

struct Command {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum CMDMODE : int {
    SINGLEROTOR = 0,
    THRUSTRATE = 1,
  };

  Command();


  //
  bool valid() const;
  bool isSingleRotorThrusts() const;
  bool isThrustRates() const;

  //
  bool setZeros(void);
  bool setCmdVector(const Ref<Vector<4>> cmd);
  bool setCmdMode(const int cmd_mode);
  bool setCmdConstraints(const Ref<Vector<4>> cmd);

  /// time in [s]
  Scalar t{NAN};

  /// Single rotor thrusts in [N]
  Vector<4> thrusts{NAN, NAN, NAN, NAN};

  /// Collective mass-normalized thrust in [m/s^2]
  Scalar collective_thrust{NAN};

  /// Bodyrates in [rad/s]
  Vector<3> omega{NAN, NAN, NAN};

  ///
  int cmd_mode;
};

}  // namespace flightlib