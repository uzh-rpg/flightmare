#pragma once

#include "flightlib/common/types.hpp"

namespace flightlib {

class DynamicsBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using DynamicsFunction =
    std::function<bool(const Ref<const Vector<>>, Ref<Vector<>>)>;

  DynamicsBase();
  virtual ~DynamicsBase();

  // public get function
  virtual DynamicsFunction getDynamicsFunction() const = 0;

 private:
};

}  // namespace flightlib
