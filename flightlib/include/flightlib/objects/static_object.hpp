#pragma once

#include "flightlib/common/rigid_state.hpp"
#include "flightlib/common/types.hpp"

namespace flightlib {
class StaticObject {
 public:
  StaticObject(std::string id, std::string prefab_id)
    : id_(id), prefab_id_(prefab_id){};
  virtual ~StaticObject(){};

  // publich get functions
  virtual Vector<3> getPos(void) { return state_.p; };
  virtual Quaternion getQuat(void) { return state_.q(); };
  virtual Vector<3> getSize(void) { return size_; };
  virtual Vector<3> getScale(void) { return scale_; };

  // public get functions
  const std::string getID(void) { return id_; };
  const std::string getPrefabID(void) { return prefab_id_; };

  // publich set functions
  inline void setPosition(const Vector<3>& position) { state_.p = position; };
  inline void setRotation(const Quaternion& quaternion) {
    state_.q(quaternion);
  };
  inline void setSize(const Vector<3>& size) { size_ = size; };
  inline void setScale(const Vector<3>& scale) { scale_ = scale; };


 protected:
  const std::string id_;
  const std::string prefab_id_;

  RigidState state_;
  Vector<3> size_;
  Vector<3> scale_;
};

}  // namespace flightlib