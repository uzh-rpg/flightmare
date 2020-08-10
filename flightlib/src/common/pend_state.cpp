#include "flightlib/common/pend_state.hpp"

namespace flightlib {

PendState::PendState() {}

PendState::PendState(const Vector<IDX::SIZE>& x, const Scalar t) : x(x), t(t) {}

PendState::PendState(const PendState& state) : x(state.x), t(state.t) {}

PendState::~PendState() {}

Quaternion PendState::q() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ));
}

void PendState::q(const Quaternion quaternion) {
  x(IDX::ATTW) = quaternion.w();
  x(IDX::ATTX) = quaternion.x();
  x(IDX::ATTY) = quaternion.y();
  x(IDX::ATTZ) = quaternion.z();
}

Matrix<3, 3> PendState::R() const {
  return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ)).toRotationMatrix();
}

void PendState::setZero() {
  t = 0.0;
  x.setZero();
  x(ATTW) = 1.0;
}

std::ostream& operator<<(std::ostream& os, const PendState& state) {
  os.precision(3);
  os << "State at " << state.t << "s: [" << state.x.transpose() << "]";
  os.precision();
  return os;
}

}  // namespace flightlib