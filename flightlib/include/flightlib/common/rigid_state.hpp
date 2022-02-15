#pragma once

#include <memory>

#include "flightlib/common/types.hpp"

namespace flightlib {

struct RigidState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum IDX : int {
    // position
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    // quaternion
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    // linear velocity
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    // body rate
    OME = 10,
    OMEX = 10,
    OMEY = 11,
    OMEZ = 12,
    NOME = 3,
    //
    SIZE = 13,
  };

  RigidState();
  RigidState(const Vector<IDX::SIZE>& x, const Scalar t = NAN);
  RigidState(const RigidState& state);
  ~RigidState();

  inline static int size() { return SIZE; }
  Quaternion q() const;
  void q(const Quaternion quaternion);
  Matrix<3, 3> R() const;
  void setZero();

  inline bool valid() const { return x.allFinite() && std::isfinite(t); }

  Vector<IDX::SIZE> x = Vector<IDX::SIZE>::Constant(NAN);
  Scalar t{NAN};

  // position
  Ref<Vector<3>> p{x.segment<IDX::NPOS>(IDX::POS)};
  // orientation (quaternion)
  Ref<Vector<4>> qx{x.segment<IDX::NATT>(IDX::ATT)};
  // linear velocity
  Ref<Vector<3>> v{x.segment<IDX::NVEL>(IDX::VEL)};
  // angular velocity
  Ref<Vector<3>> w{x.segment<IDX::NOME>(IDX::OME)};

  bool operator==(const RigidState& rhs) const {
    return t == rhs.t && x.isApprox(rhs.x, 1e-5);
  }

  friend std::ostream& operator<<(std::ostream& os, const RigidState& state);
};

using RS = RigidState;

}  // namespace flightlib