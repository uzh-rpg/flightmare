#include "flightlib/common/integrator_rk4.hpp"

namespace flightlib {

bool IntegratorRK4::step(const Ref<const Vector<>> initial, const Scalar dt,
                         Ref<Vector<>> final) const {
  static const Vector<4> rk4_sum_vec{1.0 / 6.0, 2.0 / 6.0, 2.0 / 6.0,
                                     1.0 / 6.0};
  Matrix<> k = Matrix<>::Zero(initial.rows(), 4);

  final = initial;

  // k_1
  if (!this->dynamics_(final, k.col(0))) return false;

  // k_2
  final = initial + 0.5 * dt * k.col(0);
  if (!this->dynamics_(final, k.col(1))) return false;

  // k_3
  final = initial + 0.5 * dt * k.col(1);
  if (!this->dynamics_(final, k.col(2))) return false;

  // k_4
  final = initial + dt * k.col(2);
  if (!this->dynamics_(final, k.col(3))) return false;


  final = initial + dt * k * rk4_sum_vec;

  return true;
}

}  // namespace flightlib