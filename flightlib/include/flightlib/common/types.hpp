#pragma once

#include <Eigen/Eigen>

namespace flightlib {

// ------------ General Stuff-------------

// Define the scalar type used.
using Scalar = double;  // numpy float32

// Define frame id for unity
using FrameID = uint64_t;

// Define frame id for unity
using SceneID = size_t;

// ------------ Eigen Stuff-------------

// Define `Dynamic` matrix size.
static constexpr int Dynamic = Eigen::Dynamic;

// Using shorthand for `Matrix<ros, cols>` with scalar type.
template<int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

// Using shorthand for `Matrix<ros, cols>` with scalar type.
template<int rows = Dynamic, int cols = Dynamic>
using MatrixRowMajor = Eigen::Matrix<Scalar, rows, cols, Eigen::RowMajor>;

// Using shorthand for 'ImgMatrix<ros, cols>' with scalar type.
template<int rows = Dynamic, int cols = Dynamic>
using ImgMatrix = Eigen::Matrix<uint8_t, rows, cols>;

// Using shorthand for 'ImgMatrixRowMajor<ros, cols>' with scalar type.
template<int rows = Dynamic, int cols = Dynamic>
using ImgMatrixRowMajor = Eigen::Matrix<uint8_t, rows, cols, Eigen::RowMajor>;

// Using shorthand for 'DepthImg<ros, cols>' with scalar type.
template<int rows = Dynamic, int cols = Dynamic>
using DepthImgMatrix = Eigen::Matrix<float_t, rows, cols>;

// Using shorthand for 'DepthImgRowMajor<ros, cols>' with scalar type.
template<int rows = Dynamic, int cols = Dynamic>
using DepthImgMatrixRowMajor =
  Eigen::Matrix<float_t, rows, cols, Eigen::RowMajor>;

// Using shorthand for `Vector<ros>` with scalar type.
template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;

// Using shorthand for `ImgVector<ros>` with scalar type.
template<int rows = Dynamic>
using ImgVector = ImgMatrix<rows, 1>;

// Using shorthand for `DepthImgVector<ros>` with scalar type.
template<int rows = Dynamic>
using DepthImgVector = DepthImgMatrix<rows, 1>;

// Vector bool
template<int rows = Dynamic>
using BoolVector = Eigen::Matrix<bool, rows, 1>;

// Using shorthand for `Array<rows, cols>` with scalar type.
template<int rows = Dynamic, int cols = rows>
using Array = Eigen::Array<Scalar, rows, cols>;

// Using `SparseMatrix` with type.
using SparseMatrix = Eigen::SparseMatrix<Scalar>;

// Using SparseTriplet with type.
using SparseTriplet = Eigen::Triplet<Scalar>;

// Using `Quaternion` with type.
using Quaternion = Eigen::Quaternion<Scalar>;

// Using 'AngleAxis' with type
using AngleAxis = Eigen::AngleAxis<Scalar>;

// Using `Ref` for modifier references.
template<class Derived>
using Ref = Eigen::Ref<Derived>;

// // Using `ConstRef` for constant references.
// template<class Derived>
// using ConstRef = const Eigen::Ref<const Derived>;

// Using `Map`.
template<class Derived>
using Map = Eigen::Map<Derived>;

static constexpr Scalar Gz = -9.81;
const Vector<3> GVEC{0.0, 0.0, Gz};

}  // namespace flightlib
