
#include "flightlib/objects/static_gate.hpp"

namespace flightlib {

StaticGate::StaticGate(const std::string& id, const std::string& prefab_id)
  : StaticObject(id, prefab_id) {
  id_ = id;
  prefab_id_ = prefab_id;

  //
  gate_state_.setZero();
  size_ << 1.0, 1.0, 1.0;
}

StaticGate::~StaticGate() {}

Vector<3> StaticGate::getPos(void) { return gate_state_.p; }

Quaternion StaticGate::getQuat(void) { return gate_state_.q(); }

Vector<3> StaticGate::getSize(void) { return size_; }

Vector<3> StaticGate::getScale(void) { return scale_; }


}  // namespace flightlib