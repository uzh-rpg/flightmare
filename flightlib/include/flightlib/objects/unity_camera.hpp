#pragma once

#include "flightlib/common/types.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

namespace flightlib {

class UnityCamera : RGBCamera {
 public:
  UnityCamera();
  ~UnityCamera();

 private:
};
}  // namespace flightlib
