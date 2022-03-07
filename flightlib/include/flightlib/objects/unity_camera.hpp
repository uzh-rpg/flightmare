#pragma once

#include "flightlib/common/types.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

namespace flightlib {

class UnityCamera : RGBCamera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  UnityCamera();
  ~UnityCamera();

 private:
};
}  // namespace flightlib
