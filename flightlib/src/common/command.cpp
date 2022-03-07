#include "flightlib/common/command.hpp"


namespace flightlib {

Command::Command() : t(0.0), cmd_mode(1) {}

bool Command::setCmdMode(const int mode) {
  if (mode != 0 && mode != 1) {
    return false;
  }
  cmd_mode = mode;
  return true;
}

bool Command::valid() const {
  return std::isfinite(t) && ((std::isfinite(collective_thrust) &&
                               omega.allFinite() && (cmd_mode == 1)) ||
                              (thrusts.allFinite() && (cmd_mode == 0)));
}

bool Command::isSingleRotorThrusts() const {
  return (cmd_mode == 0) && thrusts.allFinite();
}

bool Command::isThrustRates() const {
  return (cmd_mode == 1) &&
         (std::isfinite(collective_thrust) && omega.allFinite());
}


bool Command::setZeros() {
  t = 0.0;
  if (cmd_mode == 0) {
    thrusts = Vector<4>::Zero();
  } else if (cmd_mode == 1) {
    collective_thrust = 0;
    omega = Vector<3>::Zero();
  } else {
    return false;
  }
  return true;
}

bool Command::setCmdVector(const Ref<Vector<4>> cmd) {
  if (cmd_mode == 0) {
    thrusts = cmd;
  } else if (cmd_mode == 1) {
    collective_thrust = cmd(0);
    omega = cmd.segment<3>(1);
  } else {
    return false;
  }
  return true;
}

}  // namespace flightlib