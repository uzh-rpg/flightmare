#pragma once

#include "flightlib/common/types.hpp"

namespace flightlib {
class SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorBase();
  virtual ~SensorBase();

 private:
};

}  // namespace flightlib
