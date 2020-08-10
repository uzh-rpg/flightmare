#pragma once

#include <memory>

#include "flightlib/common/types.hpp"

namespace flightlib {

struct PendState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum IDX : int {
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    OME = 10,
    OMEX = 10,
    OMEY = 11,
    OMEZ = 12,
    NOME = 3,
    ACC = 13,
    ACCX = 13,
    ACCY = 14,
    ACCZ = 15,
    NACC = 3,
    TAU = 16,
    TAUX = 16,
    TAUY = 17,
    TAUZ = 18,
    NTAU = 3,
    BOME = 19,
    BOMEX = 19,
    BOMEY = 20,
    BOMEZ = 21,
    NBOME = 3,
    BACC = 22,
    BACCX = 22,
    BACCY = 23,
    BACCZ = 24,
    NBACC = 3,
    SIZE = 25,
    DYN = 19
  };

  PendState();
  PendState(const Vector<IDX::SIZE>& x, const Scalar t = NAN);
  PendState(const PendState& state);
  ~PendState();

  inline static int size() { return SIZE; }
  Quaternion q() const;
  void q(const Quaternion quaternion);
  Matrix<3, 3> R() const;
  void setZero();

  inline bool valid() const { return x.allFinite() && std::isfinite(t); }

  Vector<IDX::SIZE> x = Vector<IDX::SIZE>::Constant(NAN);
  Scalar t{NAN};

  Ref<Vector<3>> p{x.segment<IDX::NPOS>(IDX::POS)};
  Ref<Vector<4>> qx{x.segment<IDX::NATT>(IDX::ATT)};
  Ref<Vector<3>> v{x.segment<IDX::NVEL>(IDX::VEL)};
  Ref<Vector<3>> w{x.segment<IDX::NOME>(IDX::OME)};
  Ref<Vector<3>> a{x.segment<IDX::NACC>(IDX::ACC)};
  Ref<Vector<3>> tau{x.segment<IDX::NTAU>(IDX::TAU)};
  Ref<Vector<3>> bw{x.segment<IDX::NBOME>(IDX::BOME)};
  Ref<Vector<3>> ba{x.segment<IDX::NBACC>(IDX::BACC)};

  bool operator==(const PendState& rhs) const {
    return t == rhs.t && x.isApprox(rhs.x, 1e-5);
  }

  friend std::ostream& operator<<(std::ostream& os, const PendState& state);
};

using PS = PendState;

}  // namespace flightlib