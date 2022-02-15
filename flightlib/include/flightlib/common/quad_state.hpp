#pragma once

#include <memory>

#include "flightlib/common/types.hpp"

namespace flightlib {

struct QuadState {
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
    // linear acceleration
    ACC = 13,
    ACCX = 13,
    ACCY = 14,
    ACCZ = 15,
    NACC = 3,
    // body-torque
    TAU = 16,
    TAUX = 16,
    TAUY = 17,
    TAUZ = 18,
    NTAU = 3,
    //
    BOME = 19,
    BOMEX = 19,
    BOMEY = 20,
    BOMEZ = 21,
    NBOME = 3,
    //
    BACC = 22,
    BACCX = 22,
    BACCY = 23,
    BACCZ = 24,
    NBACC = 3,
    //
    SIZE = 25,
    NDYM = 19
  };

  QuadState();
  QuadState(const Vector<IDX::SIZE>& x, const Scalar t = NAN);
  QuadState(const QuadState& state);
  ~QuadState();

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
  // linear accleration
  Ref<Vector<3>> a{x.segment<IDX::NACC>(IDX::ACC)};
  // body torque
  Ref<Vector<3>> tau{x.segment<IDX::NTAU>(IDX::TAU)};
  //
  Ref<Vector<3>> bw{x.segment<IDX::NBOME>(IDX::BOME)};
  //
  Ref<Vector<3>> ba{x.segment<IDX::NBACC>(IDX::BACC)};

  bool operator==(const QuadState& rhs) const {
    return t == rhs.t && x.isApprox(rhs.x, 1e-5);
  }

  friend std::ostream& operator<<(std::ostream& os, const QuadState& state);
};

using QS = QuadState;

}  // namespace flightlib