#pragma once

#include "flightlib/common/rigid_state.hpp"
#include "flightlib/objects/static_object.hpp"

namespace flightlib {
class StaticGate : public StaticObject {
 public:
  StaticGate(const std::string& id, const std::string& prefab_id = "rpg_gate");
  ~StaticGate();

  //
  Vector<3> getPos(void) override;
  Quaternion getQuat(void) override;

  //
  Vector<3> getSize(void) override;
  Vector<3> getScale(void) override;


  // publich set functions
  inline void setPosition(const Vector<3>& position) {
    gate_state_.p = position;
  };
  inline void setRotation(const Quaternion& quaternion) {
    gate_state_.q(quaternion);
  };
  inline void setSize(const Vector<3>& size) { size_ = size; };
  inline void setScale(const Vector<3>& scale) { scale_ = scale; };


 private:
  std::string id_;
  std::string prefab_id_;

  RigidState gate_state_;
  Vector<3> size_;
  Vector<3> scale_;
};

}  // namespace flightlib
