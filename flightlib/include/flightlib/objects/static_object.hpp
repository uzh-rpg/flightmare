#pragma once

#include "flightlib/common/types.hpp"

namespace flightlib {
class StaticObject {
 public:
  StaticObject(const std::string& id, const std::string& prefab_id)
    : id_(id), prefab_id_(prefab_id){};
  virtual ~StaticObject(){};

  // public set functions
  virtual void setPosition(const Vector<3>& position) { position_ = position; };
  virtual void setQuaternion(const Quaternion& quaternion) {
    quat_ = quaternion;
  };
  virtual void setSize(const Vector<3>& size) { size_ = size; };

  // public get functions
  virtual Vector<3> getPosition(void) { return position_; };
  virtual Quaternion getQuaternion(void) { return quat_; };
  virtual Vector<3> getSize(void) { return size_; };
  const std::string& getID(void) { return id_; };
  const std::string& getPrefabID(void) { return prefab_id_; };

 private:
  std::string id_;
  std::string prefab_id_;

 protected:
  Vector<3> position_{0.0, 0.0, 0.0};
  Quaternion quat_{1.0, 0.0, 0.0, 0.0};
  Vector<3> size_{1.0, 1.0, 1.0};
};

}  // namespace flightlib
