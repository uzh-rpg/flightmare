#pragma once

#include "flightlib/objects/static_object.hpp"

namespace flightlib {
class StaticGate : public StaticObject {
 public:
  StaticGate(const std::string& id, const std::string& prefab_id = "rpg_gate")
    : StaticObject(id, prefab_id) {}
  ~StaticGate() {}
};

}  // namespace flightlib
