#include <gtest/gtest.h>

#include "flightlib/common/math.hpp"
#include "flightlib/common/types.hpp"

using namespace flightlib;

TEST(EigenChecks, EigenVersionOutput) {
  std::printf("Eigen Version: %d.%d.%d\n", EIGEN_WORLD_VERSION,
              EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
}

TEST(EigenChecks, EigenQuaternionSequence) {
  Quaternion q1, q2;

  q1.coeffs() = Vector<4>::Random();
  q1.normalize();
  q2.coeffs() = Vector<4>::Random();
  q2.normalize();

  const Matrix<3, 3> R1 = q1.toRotationMatrix();
  const Matrix<3, 3> R2 = q2.toRotationMatrix();

  const Quaternion q3 = q2 * q1;
  const Matrix<3, 3> Rq3 = q3.toRotationMatrix();

  const Matrix<3, 3> R3 = R2 * R1;

  EXPECT_TRUE(R3.isApprox(Rq3))
    << "Quternion multiplication sequence does not follow rotation!";

  const Vector<3> r = Vector<3>::Random();
  const Vector<3> r_R3 = R3 * r;

  const Vector<3> r_Rq3 = q3.toRotationMatrix() * r;
  EXPECT_TRUE(r_Rq3.isApprox(r_R3));

  const Vector<3> r_q3 = q3 * r;
  EXPECT_TRUE(r_q3.isApprox(r_R3));

  const Vector<3> r_q21 = (q2 * q1) * r;
  EXPECT_TRUE(r_q21.isApprox(r_R3));

  const Vector<3> r_q2q1 = q2 * (q1 * r);
  EXPECT_TRUE(r_q2q1.isApprox(r_R3));

  const Vector<3> r_R2R1 = R2 * R1 * r;
  EXPECT_TRUE(r_R2R1.isApprox(r_R3));
}

TEST(EigenChecks, EigenQuaternionRotationDirection) {
  const Quaternion qz90(sqrt(0.5), 0, 0, sqrt(0.5));
  const Quaternion qy90(sqrt(0.5), 0, sqrt(0.5), 0);
  const Quaternion qy90z90 = qz90 * qy90;
  const Matrix<3, 3> Rz90 =
    (Matrix<3, 3>() << 0, -1, 0, 1, 0, 0, 0, 0, 1).finished();
  const Matrix<3, 3> Ry90 =
    (Matrix<3, 3>() << 0, 0, 1, 0, 1, 0, -1, 0, 0).finished();
  const Matrix<3, 3> Ry90z90 = Rz90 * Ry90;

  const Vector<3> r(1, 0, 0);
  const Vector<3> r_Rz90expected(0, 1, 0);
  const Vector<3> r_Ry90z90expected(0, 0, -1);

  EXPECT_TRUE((Rz90 * r).isApprox(r_Rz90expected));
  EXPECT_TRUE((qz90.toRotationMatrix() * r).isApprox(r_Rz90expected));
  EXPECT_TRUE((qz90 * r).isApprox(r_Rz90expected));
  EXPECT_TRUE(qz90.toRotationMatrix().isApprox(Rz90));

  EXPECT_TRUE((Ry90z90 * r).isApprox(r_Ry90z90expected))
    << "Expected: " << r_Ry90z90expected.transpose() << std::endl
    << "Actual:   " << (Ry90z90 * r).transpose() << std::endl;
  EXPECT_TRUE((qy90z90.toRotationMatrix() * r).isApprox(r_Ry90z90expected))
    << "Expected: " << r_Ry90z90expected.transpose() << std::endl
    << "Actual:   " << (qy90z90.toRotationMatrix() * r).transpose()
    << std::endl;
  EXPECT_TRUE((qy90z90 * r).isApprox(r_Ry90z90expected))
    << "Expected: " << r_Ry90z90expected.transpose() << std::endl
    << "Actual:   " << (qy90z90 * r).transpose() << std::endl;
  EXPECT_TRUE(qy90z90.toRotationMatrix().isApprox(Ry90z90))
    << "Expected: " << std::endl
    << Ry90z90 << std::endl
    << "Actual:   " << std::endl
    << qy90z90.toRotationMatrix() << std::endl;
}

TEST(EigenChecks, QuaternionCrossMatrix) {
  const Vector<4> v1 = Vector<4>::Random().normalized();
  const Vector<4> v2 = Vector<4>::Random().normalized();

  EXPECT_NEAR(v1.norm(), 1.0, 1e-6);
  EXPECT_NEAR(v2.norm(), 1.0, 1e-6);

  const Quaternion q1(v1(0), v1(1), v1(2), v1(3));
  const Quaternion q2(v2(0), v2(1), v2(2), v2(3));

  const Quaternion q1q2 = q1 * q2;
  const Quaternion q2q1 = q2 * q1;

  const Vector<4> vq1q2 = (Vector<4>() << q1q2.w(), q1q2.vec()).finished();
  const Vector<4> vq2q1 = (Vector<4>() << q2q1.w(), q2q1.vec()).finished();

  const Vector<4> Qleftq1_v2 = Q_left(q1) * v2;
  const Vector<4> Qrightq2_v1 = Q_right(q2) * v1;

  const Vector<4> Qleftq2_v1 = Q_left(q2) * v1;
  const Vector<4> Qrightq1_v2 = Q_right(q1) * v2;

  EXPECT_TRUE(Qleftq1_v2.isApprox(vq1q2));
  EXPECT_TRUE(Qrightq2_v1.isApprox(vq1q2));
  EXPECT_TRUE(Qleftq2_v1.isApprox(vq2q1));
  EXPECT_TRUE(Qrightq1_v2.isApprox(vq2q1));
}

TEST(EigenChecks, MatrixColumnwiseDotProduct) {
  static constexpr size_t N = 64;
  static constexpr size_t S = 2;
  const Matrix<S, N> A = Matrix<S, N>::Random();

  Vector<N> expected = Vector<N>::Zero();
  for (size_t i = 0; i < N; ++i) expected(i) = A.col(i).transpose() * A.col(i);

  Vector<N> dotproduct = (A.cwiseProduct(A)).colwise().sum();

  EXPECT_TRUE(dotproduct.isApprox(expected));
}