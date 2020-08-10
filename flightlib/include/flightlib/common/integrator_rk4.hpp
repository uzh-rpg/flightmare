#pragma once

#include "flightlib/common/integrator_base.hpp"

namespace flightlib {

class IntegratorRK4 : public IntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using IntegratorBase::DynamicsFunction;
  using IntegratorBase::IntegratorBase;

  bool step(const Ref<const Vector<>> initial, const Scalar dt,
            Ref<Vector<>> final) const;
};

}  // namespace flightlib