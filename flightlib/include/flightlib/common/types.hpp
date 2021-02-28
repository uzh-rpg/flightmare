#pragma once

#include <eigen3/Eigen/Eigen>
#include <memory>
#include <opencv2/core/core.hpp>

// #include <cstdio>
// #include <stdint.h>
// #include <cinttypes>
// #include <ze/common/transformation.hpp>
#define ImageFloatType float


namespace flightlib {

// ------------ General Stuff-------------

// Define the scalar type used.
using Scalar = float;  // numpy float32


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

// Using shorthand for `Vector<ros>` with scalar type.
template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;

// Vector bool
template<int rows = Dynamic>
using BoolVector = Eigen::Matrix<bool, -1, 1>;

// Using shorthand for `Array<rows, cols>` with scalar type.
template<int rows = Dynamic, int cols = rows>
using Array = Eigen::Array<Scalar, rows, cols>;

// Using `SparseMatrix` with type.
using SparseMatrix = Eigen::SparseMatrix<Scalar>;

// Using SparseTriplet with type.
using SparseTriplet = Eigen::Triplet<Scalar>;

// Using `Quaternion` with type.
using Quaternion = Eigen::Quaternion<Scalar>;

// Using `Ref` for modifier references.
template<class Derived>
using Ref = Eigen::Ref<Derived>;

// Using time for events
// using Time = ze::int64_t;
// using uint16_t = ze::uint16_t;
// using uint16_t =std::unit16_t;
// using Time = std::int64_t;


// // Using `ConstRef` for constant references.
// template<class Derived>
// using ConstRef = const Eigen::Ref<const Derived>;

// Using `Map`.
template<class Derived>
using Map = Eigen::Map<Derived>;

static constexpr Scalar Gz = -9.81;
const Vector<3> GVEC{0.0, 0.0, Gz};

// struct Event
// {
//   Event(uint16_t x, uint16_t y, Time t, bool pol)
//     : x(x),
//       y(y),
//       t(t),
//       pol(pol)
//   {

//   }

//   uint16_t x;
//   uint16_t y;
//   Time t;
//   bool pol;
// };

struct Event_t {
  int coord_x;
  int coord_y;
  int polarity;
  int32_t time;
};

struct TimeMessage_t {
  int64_t current_time;
  int64_t next_timestep;
  bool rgb_frame;
};
using EventsVector = std::vector<Event_t>;

using Image = cv::Mat_<ImageFloatType>;
using ImagePtr = std::shared_ptr<Image>;
using ImagePtrVector = std::vector<ImagePtr>;

using RGBImage = cv::Mat;
using RGBImagePtr = std::shared_ptr<RGBImage>;
using RGBImagePtrVector = std::vector<RGBImagePtr>;




}  // namespace flightlib
