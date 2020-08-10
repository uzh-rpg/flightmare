#include "flightlib/common/integrator_base.hpp"

namespace flightlib {

IntegratorBase::IntegratorBase(IntegratorBase::DynamicsFunction function,
                               const Scalar dt_max)
  : dynamics_(function), dt_max_(dt_max) {}

bool IntegratorBase::integrate(const QuadState& initial,
                               QuadState* const final) const {
  if (std::isnan(initial.t) || std::isnan(final->t)) return false;
  if (initial.t >= final->t) return false;
  return integrate(initial.x, final->t - initial.t, final->x);
}

bool IntegratorBase::integrate(const Ref<const Vector<>> initial,
                               const Scalar dt, Ref<Vector<>> final) const {
  Scalar dt_remaining = dt;
  Vector<> state = initial;

  do {
    const Scalar dt_this = std::min(dt_remaining, dt_max_);
    if (!step(state, dt_this, final)) return false;
    state = final;
    dt_remaining -= dt_this;
  } while (dt_remaining > 0.0);

  return true;
}

Scalar IntegratorBase::dtMax() const { return dt_max_; }

}  // namespace flightlib