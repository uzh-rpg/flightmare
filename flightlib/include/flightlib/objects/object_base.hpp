#pragma once

#include "flightlib/common/types.hpp"

namespace flightlib {

class ObjectBase {
 public:
  ObjectBase();
  virtual ~ObjectBase();

  // reset
  virtual bool reset(void) = 0;

  // run
  virtual bool run(const Scalar dt) = 0;
};

}  // namespace flightlib
