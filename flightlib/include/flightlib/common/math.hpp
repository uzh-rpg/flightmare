#pragma once

#include "flightlib/common/types.hpp"

namespace flightlib {

Matrix<3, 3> skew(const Vector<3>& v);

Matrix<4, 4> Q_left(const Quaternion& q);

Matrix<4, 4> Q_right(const Quaternion& q);

Matrix<4, 3> qFromQeJacobian(const Quaternion& q);

Matrix<4, 4> qConjugateJacobian();

Matrix<3, 3> qeRotJacobian(const Quaternion& q, const Matrix<3, 1>& t);

Matrix<3, 3> qeInvRotJacobian(const Quaternion& q, const Matrix<3, 1>& t);

void matrixToTripletList(const SparseMatrix& matrix,
                         std::vector<SparseTriplet>* const list,
                         const int row_offset = 0, const int col_offset = 0);

void matrixToTripletList(const Matrix<>& matrix,
                         std::vector<SparseTriplet>* const list,
                         const int row_offset = 0, const int col_offset = 0);

void insert(const SparseMatrix& from, SparseMatrix* const into,
            const int row_offset = 0, const int col_offset = 0);

void insert(const Matrix<>& from, SparseMatrix* const into,
            const int row_offset = 0, const int col_offset = 0);

void insert(const Matrix<>& from, Matrix<>* const into,
            const int row_offset = 0, const int col_offset = 0);

void quaternionToEuler(const Quaternion& quat, Ref<Vector<3>> euler);

std::vector<Scalar> transformationRos2Unity(const Matrix<4, 4>& ros_tran_mat);

std::vector<Scalar> positionRos2Unity(const Vector<3>& ros_pos_vec);

std::vector<Scalar> quaternionRos2Unity(const Quaternion& ros_quat);

std::vector<Scalar> scalarRos2Unity(const Vector<3>& ros_scale);

}  // namespace flightlib
