#pragma once

#include "flightlib/common/integrator_base.hpp"

namespace flightlib {

class IntegratorEuler : public IntegratorBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using IntegratorBase::DynamicsFunction;
  using IntegratorBase::IntegratorBase;

  bool step(const Ref<const Vector<>> initial, const Scalar dt,
            Ref<Vector<>> final) const;
};

}  // namespace flightlib