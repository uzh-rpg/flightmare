#pragma once

#include "flightlib/objects/static_object.hpp"

namespace flightlib {
class StaticGate : public StaticObject {
 public:
  StaticGate(std::string id, std::string prefab_id);
  ~StaticGate(){};

  // publich set functions
  void setPosition(const Vector<3>& position) { position_ = position; };
  void setRotation(const Quaternion& quaternion) { quat_ = quaternion; };
  void setSize(const Vector<3>& size) { size_ = size; };

  // publich get functions
  Vector<3> getPos(void) { return position_; };
  Quaternion getQuat(void) { return quat_; };
  Vector<3> getSize(void) { return size_; };

 private:
  std::string id_;
  std::string prefab_id_;

  Vector<3> position_{0.0, 0.0, 0.0};
  Quaternion quat_{1.0, 0.0, 0.0, 0.0};
  Vector<3> size_{1.0, 1.0, 1.0};
};

}  // namespace flightlib