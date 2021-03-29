#pragma once

#include "flightlib/objects/static_object.hpp"

namespace flightlib {
class StaticGate : public StaticObject {
 public:
  StaticGate(std::string id) : StaticObject(id, "rpg_gate") {}
  ~StaticGate() {}
};

}  // namespace flightlib
