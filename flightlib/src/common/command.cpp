#include "flightlib/common/command.hpp"


namespace flightlib {

Command::Command()
  : t(0.0),
    thrusts(0.0, 0.0, 0.0, 0.0),
    collective_thrust(0.0),
    omega(0.0, 0.0, 0.0),
    cmd_mode(1) {}

Command::~Command() {}

bool Command::setCmdMode(const int mode) {
  if (mode != 0 && mode != 1) {
    return false;
  }
  cmd_mode = mode;
  return true;
}

bool Command::valid() const {
  return std::isfinite(t) &&
         ((std::isfinite(collective_thrust) && omega.allFinite() &&
           (cmd_mode == quadcmd::THRUSTRATE)) ||
          (thrusts.allFinite() && (cmd_mode == quadcmd::SINGLEROTOR)));
}

bool Command::isSingleRotorThrusts() const {
  return (cmd_mode == quadcmd::SINGLEROTOR) && thrusts.allFinite();
}

bool Command::isThrustRates() const {
  return (cmd_mode == quadcmd::THRUSTRATE) &&
         (std::isfinite(collective_thrust) && omega.allFinite());
}


void Command::setZeros() {
  t = 0.0;
  if (cmd_mode == quadcmd::SINGLEROTOR) {
    thrusts = Vector<4>::Zero();
  } else if (cmd_mode == quadcmd::THRUSTRATE) {
    collective_thrust = 0;
    omega = Vector<3>::Zero();
  }
}

void Command::setCmdVector(const Vector<4>& cmd) {
  if (cmd_mode == quadcmd::SINGLEROTOR) {
    thrusts = cmd;
  } else if (cmd_mode == quadcmd::THRUSTRATE) {
    collective_thrust = cmd(0);
    omega = cmd.segment<3>(1);
  }
}

}  // namespace flightlib