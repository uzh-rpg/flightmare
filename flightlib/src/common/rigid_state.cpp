
#include "flightlib/common/rigid_state.hpp"

namespace flightlib {

RigidState::RigidState() { setZero(); }

RigidState::RigidState(const Vector<IDX::SIZE>& x, const Scalar t)
  : x(x), t(t) {}

RigidState::RigidState(const RigidState& state) : x(state.x), t(state.t) {}

RigidState::~RigidState() {}

Quaternion RigidState::q() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ));
}

void RigidState::q(const Quaternion quaternion) {
  x(IDX::ATTW) = quaternion.w();
  x(IDX::ATTX) = quaternion.x();
  x(IDX::ATTY) = quaternion.y();
  x(IDX::ATTZ) = quaternion.z();
}

Matrix<3, 3> RigidState::R() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ)).toRotationMatrix();
}

void RigidState::setZero() {
  t = 0.0;
  x.setZero();
  x(ATTW) = 1.0;
}

std::ostream& operator<<(std::ostream& os, const RigidState& state) {
  os.precision(3);
  os << "State at " << state.t << "s: [" << state.x.transpose() << "]";
  os.precision();
  return os;
}

}  // namespace flightlib