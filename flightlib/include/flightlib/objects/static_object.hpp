#pragma once

#include "flightlib/common/types.hpp"

namespace flightlib {
class StaticObject {
 public:
  StaticObject(std::string id, std::string prefab_id)
    : id_(id), prefab_id_(prefab_id){};
  virtual ~StaticObject(){};

  // publich get functions
  virtual Vector<3> getPos(void) = 0;
  virtual Quaternion getQuat(void) = 0;
  virtual Vector<3> getSize(void) = 0;
  virtual Vector<3> getScale(void) = 0;

  // public get functions
  const std::string getID(void) { return id_; };
  const std::string getPrefabID(void) { return prefab_id_; };


 protected:
  const std::string id_;
  const std::string prefab_id_;
};

}  // namespace flightlib