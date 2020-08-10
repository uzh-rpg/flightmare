#pragma once

#include "flightlib/common/math.hpp"
#include "flightlib/common/pend_state.hpp"
#include "flightlib/dynamics/dynamics_base.hpp"

namespace flightlib {

class PendulumDynamics : DynamicsBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  PendulumDynamics();
  ~PendulumDynamics();

  //
  bool dState(const PendState& state, PendState* derivative) const;
  bool dState(const Ref<const Vector<PendState::SIZE>> state,
              Ref<Vector<PendState::SIZE>> derivative) const;


 private:
}

}  // namespace flightlib
