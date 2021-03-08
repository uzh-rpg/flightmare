#include "flightlib/objects/static_gate.hpp"

namespace flightlib {

StaticGate::StaticGate(std::string id, std::string prefab_id)
  : StaticObject(id, prefab_id) {
  id_ = id;
  prefab_id_ = prefab_id;
};

}  // namespace flightlib