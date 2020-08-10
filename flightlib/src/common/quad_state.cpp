#include "flightlib/common/quad_state.hpp"

namespace flightlib {

QuadState::QuadState() {}

QuadState::QuadState(const Vector<IDX::SIZE>& x, const Scalar t) : x(x), t(t) {}

QuadState::QuadState(const QuadState& state) : x(state.x), t(state.t) {}

QuadState::~QuadState() {}

Quaternion QuadState::q() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ));
}

void QuadState::q(const Quaternion quaternion) {
  x(IDX::ATTW) = quaternion.w();
  x(IDX::ATTX) = quaternion.x();
  x(IDX::ATTY) = quaternion.y();
  x(IDX::ATTZ) = quaternion.z();
}

Matrix<3, 3> QuadState::R() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ)).toRotationMatrix();
}

void QuadState::setZero() {
  t = 0.0;
  x.setZero();
  x(ATTW) = 1.0;
}

std::ostream& operator<<(std::ostream& os, const QuadState& state) {
  os.precision(3);
  os << "State at " << state.t << "s: [" << state.x.transpose() << "]";
  os.precision();
  return os;
}

}  // namespace flightlib