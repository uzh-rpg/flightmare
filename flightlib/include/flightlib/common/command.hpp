
#pragma once

#include <cmath>

#include "flightlib/common/types.hpp"

namespace flightlib {

struct Command {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Command();

  Command(const Scalar t, const Scalar thrust, const Vector<3>& omega);

  Command(const Scalar t, const Vector<4>& thrusts);

  bool valid() const;
  bool isSingleRotorThrusts() const;
  bool isRatesThrust() const;

  /// time in [s]
  Scalar t{NAN};

  /// Collective mass-normalized thrust in [m/s^2]
  Scalar collective_thrust{NAN};

  /// Bodyrates in [rad/s]
  Vector<3> omega{NAN, NAN, NAN};

  /// Single rotor thrusts in [N]
  Vector<4> thrusts{NAN, NAN, NAN, NAN};
};

}  // namespace flightlib