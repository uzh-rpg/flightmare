#pragma once

#include <functional>

#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"

namespace flightlib {

class IntegratorBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using DynamicsFunction =
    std::function<bool(const Ref<const Vector<>>, Ref<Vector<>>)>;
  IntegratorBase(DynamicsFunction function, const Scalar dt_max = 1e-3);

  bool integrate(const QuadState& initial, QuadState* const final) const;

  bool integrate(const Ref<const Vector<>> initial, const Scalar dt,
                 Ref<Vector<>> final) const;

  virtual bool step(const Ref<const Vector<>> initial, const Scalar dt,
                    Ref<Vector<>> final) const = 0;

  Scalar dtMax() const;

 protected:
  DynamicsFunction dynamics_;
  Scalar dt_max_;
};

}  // namespace flightlib