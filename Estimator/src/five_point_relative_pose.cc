// Copyright (C) 2013 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "five_point_relative_pose.h"

#include <Eigen/Dense>

#include <cmath>
#include <ctime>
#include <vector>

#include "gauss_jordan.h"
#include "polynomial.h"
#include "util.h"

namespace theia {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Matrix;
using Eigen::RowVector3d;
using Eigen::RowVector4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

typedef Matrix<double, 3, 3, Eigen::RowMajor> RowMatrix3d;

namespace {
// Multiplies two polynomials over the same variable.
template <int n1, int n2>
Matrix<double, 1, n1 + n2 - 1> MultiplyPoly(const Matrix<double, 1, n1>& a,
                                            const Matrix<double, 1, n2>& b) {
  Matrix<double, 1, n1 + n2 - 1> poly = Matrix<double, 1, n1 + n2 - 1>::Zero();
  for (int i = 0; i < a.cols(); i++)
    for (int j = 0; j < b.cols(); j++)
      poly[i + j] += a[i] * b[j];

  return poly;
}

// Evaluates a given polynomial at the value x.
template <int n>
double EvaluatePoly(const Matrix<double, 1, n>& poly, double x) {
  double val = 0;
  for (int i = poly.cols() - 1; i > 0; i--) {
    val += poly[i];
    val *= x;
  }
  val += poly[0];
  return val;
}

// Multiply two degree one polynomials of variables x, y, z.
// E.g. p1 = a[0]x + a[1]y + a[2]z + a[3]
// x^2 y^2 z^2 xy xz yz x y z 1
Matrix<double, 1, 10> MultiplyDegOnePoly(const RowVector4d& a,
                                         const RowVector4d& b) {
  Matrix<double, 1, 10> output;
  output(0) = a(0) * b(0);
  output(1) = a(1) * b(1);
  output(2) = a(2) * b(2);
  output(3) = a(0) * b(1) + a(1) * b(0);
  output(4) = a(0) * b(2) + a(2) * b(0);
  output(5) = a(1) * b(2) + a(2) * b(1);
  output(6) = a(0) * b(3) + a(3) * b(0);
  output(7) = a(1) * b(3) + a(3) * b(1);
  output(8) = a(2) * b(3) + a(3) * b(2);
  output(9) = a(3) * b(3);
  return output;
}

// Multiply a 2 deg poly (in x, y, z) and a one deg poly.
// x^3 y^3 x^2y xy^2 x^2z x^2 y^2z y^2 xyz xy | z^2x zx x z^2y zy y z^3 z^2 z 1
// NOTE: after the | all are variables along z.
Matrix<double, 1, 20> MultiplyDegTwoDegOnePoly(const Matrix<double, 1, 10>& a,
                                               const RowVector4d& b) {
  Matrix<double, 1, 20> output;
  output(0) = a(0) * b(0);
  output(1) = a(1) * b(1);
  output(2) = a(0) * b(1) + a(3) * b(0);
  output(3) = a(1) * b(0) + a(3) * b(1);
  output(4) = a(0) * b(2) + a(4) * b(0);
  output(5) = a(0) * b(3) + a(6) * b(0);
  output(6) = a(1) * b(2) + a(5) * b(1);
  output(7) = a(1) * b(3) + a(7) * b(1);
  output(8) = a(3) * b(2) + a(4) * b(1) + a(5) * b(0);
  output(9) = a(3) * b(3) + a(6) * b(1) + a(7) * b(0);
  output(10) = a(2) * b(0) + a(4) * b(2);
  output(11) = a(4) * b(3) + a(8) * b(0) + a(6) * b(2);
  output(12) = a(6) * b(3) + a(9) * b(0);
  output(13) = a(2) * b(1) + a(5) * b(2);
  output(14) = a(5) * b(3) + a(8) * b(1) + a(7) * b(2);
  output(15) = a(7) * b(3) + a(9) * b(1);
  output(16) = a(2) * b(2);
  output(17) = a(2) * b(3) + a(8) * b(2);
  output(18) = a(8) * b(3) + a(9) * b(2);
  output(19) = a(9) * b(3);
  return output;
}

// Shorthand for multiplying the Essential matrix with its transpose according
// to Eq. 20 in Nister paper.
Matrix<double, 1, 10> EETranspose(
    const Matrix<double, 9, 4>& null_matrix, int i, int j) {
  return MultiplyDegOnePoly(null_matrix.row(3 * i), null_matrix.row(3 * j)) +
      MultiplyDegOnePoly(null_matrix.row(3 * i + 1),
                         null_matrix.row(3 * j + 1)) +
      MultiplyDegOnePoly(null_matrix.row(3 * i + 2),
                         null_matrix.row(3 * j + 2));
}

// Builds the 10x20 constraint matrix according to Section 3.2.2 of Nister
// paper. Constraints are built based on the singularity of the Essential
// matrix, and the trace equation (Eq. 6). This builds the 10x20 matrix such
// that the columns correspond to: x^3, yx^2, y^2x, y^3, zx^2, zyx, zy^2, z^2x,
// z^2y, z^3, x^2, yx, y^2, zx, zy, z^2, x, y, z, 1.
Matrix<double, 10, 20> BuildConstraintMatrix(
    const Matrix<double, 9, 4>& null_space) {
  Matrix<double, 10, 20> constraint_matrix;
  // Singularity constraint.
  constraint_matrix.row(0) =
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_space.row(1), null_space.row(5)) -
          MultiplyDegOnePoly(null_space.row(2), null_space.row(4)),
          null_space.row(6)) +
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_space.row(2), null_space.row(3)) -
          MultiplyDegOnePoly(null_space.row(0), null_space.row(5)),
          null_space.row(7)) +
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_space.row(0), null_space.row(4)) -
          MultiplyDegOnePoly(null_space.row(1), null_space.row(3)),
          null_space.row(8));

  // Trace Constraint. Only need to compute the upper triangular part of the
  // symmetric polynomial matrix
  Matrix<double, 1, 10> symmetric_poly[3][3];
  symmetric_poly[0][0] = EETranspose(null_space, 0, 0);
  symmetric_poly[1][1] = EETranspose(null_space, 1, 1);
  symmetric_poly[2][2] = EETranspose(null_space, 2, 2);

  Matrix<double, 1, 10> half_trace = 0.5*(symmetric_poly[0][0] +
                                          symmetric_poly[1][1] +
                                          symmetric_poly[2][2]);

  symmetric_poly[0][0] -= half_trace;
  symmetric_poly[1][1] -= half_trace;
  symmetric_poly[2][2] -= half_trace;
  symmetric_poly[0][1] = EETranspose(null_space, 0, 1);
  symmetric_poly[0][2] = EETranspose(null_space, 0, 2);
  symmetric_poly[1][0] = symmetric_poly[0][1];
  symmetric_poly[1][2] = EETranspose(null_space, 1, 2);
  symmetric_poly[2][0] = symmetric_poly[0][2];
  symmetric_poly[2][1] = symmetric_poly[1][2];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      constraint_matrix.row(3*i + j + 1) =
          MultiplyDegTwoDegOnePoly(symmetric_poly[i][0],
                                   null_space.row(j)) +
          MultiplyDegTwoDegOnePoly(symmetric_poly[i][1],
                                   null_space.row(3 + j)) +
          MultiplyDegTwoDegOnePoly(symmetric_poly[i][2],
                                   null_space.row(6 + j));
    }
  }
  return constraint_matrix;
}

// Efficient nullspace extraction based on the idea of QR decomposition on the 5
// correspondences of the epipolar constraint. Instead of QR, we use
// Gauss-Jordan for the same effect.
Matrix<double, 9, 4> EfficientNullspaceExtraction(
    const Matrix<double, 5, 9>& constraint) {
  Matrix<double, 5, 9> constraint_copy(constraint);
  GaussJordan(&constraint_copy);
  Matrix<double, 4, 9> null_space;
  null_space << constraint_copy.rightCols(4).transpose(),
      -Matrix4d::Identity();
  return null_space.transpose();
}

void EfficientSVDDecomp(const Matrix3d& essential_mat,
                        Vector3d* null_space,
                        Matrix3d rotation[4],
                        Vector3d translation[4]) {
  Matrix3d d;
  d << 0, 1, 0,
      -1, 0, 0,
      0, 0, 1;

  const Vector3d& ea = essential_mat.row(0);
  const Vector3d& eb = essential_mat.row(1);
  const Vector3d& ec = essential_mat.row(2);

  // Generate cross products.
  Matrix3d cross_products;
  cross_products << ea.cross(eb), ea.cross(ec), eb.cross(ec);

  // Choose the cross product with the largest norm (for numerical accuracy).
  const Vector3d cf_scales(cross_products.col(0).squaredNorm(),
                           cross_products.col(1).squaredNorm(),
                           cross_products.col(2).squaredNorm());
  int max_index;
  cf_scales.maxCoeff(&max_index);

  // For index 0, 1, we want ea and for index 2 we want eb.
  const int max_e_index = max_index / 2;

  // Construct v of the SVD.
  Matrix3d v = Matrix3d::Zero();
  v.col(2) = cross_products.col(max_index).normalized();
  v.col(0) = essential_mat.row(max_e_index).normalized();
  v.col(1) = v.col(2).cross(v.col(0));

  // Construct U of the SVD.
  Matrix3d u = Matrix3d::Zero();
  u.col(0) = (essential_mat * v.col(0)).normalized();
  u.col(1) = (essential_mat * v.col(1)).normalized();
  u.col(2) = u.col(0).cross(u.col(1));

  // Possible rotation configurations.
  const RowMatrix3d ra =
      Eigen::Quaterniond(u * d * v.transpose()).normalized().toRotationMatrix();
  const RowMatrix3d rb = Eigen::Quaterniond(u * d.transpose() * v.transpose())
      .normalized().toRotationMatrix();

  // Scale t to be proper magnitude. Scale factor is derived from the fact that
  // U*diag*V^t = E. We simply choose to scale it such that the last terms will
  // be equal.
  const Vector3d t = u.col(2).normalized();
  const Vector3d t_neg = -t;

  // Copy the 4 possible decompositions into the output arrays.
  rotation[0] = ra;
  translation[0] = t;
  rotation[1] = ra;
  translation[1] = t_neg;
  rotation[2] = rb;
  translation[2] = t;
  rotation[3] = rb;
  translation[3] = t_neg;

  *null_space = v.col(2);
}

void DecomposeWithIdealCorrespondence(const Vector3d& image_point1_homog,
                                      const Vector3d& image_point2_homog,
                                      const Matrix3d& essential_mat,
                                      Matrix3d* rotation,
                                      Vector3d* translation) {
  // Map the image points to vectors.
  Matrix3d candidate_rotation[4];
  Vector3d candidate_translation[4];
  Vector3d null_space;
  EfficientSVDDecomp(essential_mat, &null_space, candidate_rotation,
                     candidate_translation);

  Matrix<double, 3, 4> projection_mat;
  projection_mat.block<3, 3>(0, 0) = candidate_rotation[0];
  projection_mat.block<3, 1>(0, 3) = candidate_translation[0];

  // Compute c.
  Matrix3d temp_diag = Matrix3d::Identity();
  temp_diag(2, 2) = 0.0;
  const Vector3d c =
      image_point2_homog.cross(temp_diag * essential_mat * image_point1_homog);

  // Compute C.
  const Vector4d C = projection_mat.transpose() * c;
  const Vector4d Q(image_point1_homog(0) * C(3),
                   image_point1_homog(1) * C(3),
                   image_point1_homog(2) * C(3),
                   -(image_point1_homog.dot(C.head<3>())));
  // We only care about the sign of the depth because it informs us of the
  // direction of the point (i.e. if it is in front of the camera). Use a
  // multiply instead of divide for speed.
  const double scaled_depth_1  = Q(2) * Q(3);
  const double scaled_depth_2 = projection_mat.row(2).dot(Q) * Q(3);

  // Create the twisted pair transformation.
  const Vector4d twisted_transformation(-2.0 * null_space(0),
                                        -2.0 * null_space(1),
                                        -2.0 * null_space(2),
                                        -1.0);

  // Determine the proper configuration for the R,t decomposition.
  int best_index;
  if (scaled_depth_1 > 0 && scaled_depth_2 > 0) {
    best_index = 0;
  } else if (scaled_depth_1 < 0 && scaled_depth_2 < 0) {
    best_index = 1;
  } else if (Q(2) * twisted_transformation.dot(Q) > 0) {
    best_index = 2;
  } else {
    best_index = 3;
  }

  *rotation = candidate_rotation[best_index];
  *translation = candidate_translation[best_index];
}

}  // namespace

// Implementation of Nister from "An Efficient Solution to the Five-Point
// Relative Pose Problem"
bool FivePointRelativePose(const Vector3d image1_points[5],
                           const Vector3d image2_points[5],
                           std::vector<Matrix3d>* essentials,
                           std::vector<Matrix3d>* rotations,
                           std::vector<Vector3d>* translations ) {
  // Step 1. Create the 5x9 matrix containing epipolar constraints.
  //   Essential matrix is a linear combination of the 4 vectors spanning the
  //   null space of this matrix (found by SVD).
  Matrix<double, 5, 9> epipolar_constraint;
  for (int i = 0; i < 5; i++) {
    // Fill matrix with the epipolar constraint from q'_t*E*q = 0. Where q is
    // from the first image, and q' is from the second. Eq. 8 in the Nister
    // paper.
    epipolar_constraint.row(i) <<
        image1_points[i].x() * image2_points[i].x(),
        image1_points[i].y() * image2_points[i].x(),
        image1_points[i].z() * image2_points[i].x(),
        image1_points[i].x() * image2_points[i].y(),
        image1_points[i].y() * image2_points[i].y(),
        image1_points[i].z() * image2_points[i].y(),
        image1_points[i].x() * image2_points[i].z(),
        image1_points[i].y() * image2_points[i].z(),
        image1_points[i].z() * image2_points[i].z();
  }

  // Solve for right null space of the 5x9 matrix. NOTE: We use a
  // super-efficient method that is a variation of the QR decomposition
  // described in the Nister paper.  by roughly 5x.
  Matrix<double, 9, 4> null_space =
      EfficientNullspaceExtraction(epipolar_constraint);

  // Step 2. Expansion of the epipolar constraints Eq. 5 and 6 from Nister
  // paper.
  Matrix<double, 10, 20> constraint_matrix = BuildConstraintMatrix(null_space);

  // Step 3. Gauss-Jordan Elimination with partial pivoting on constraint
  // matrix.
  GaussJordan(&constraint_matrix);

  // Step 4. Expand determinant polynomial of 3x3 polynomial B.
  // Create matrix B. Horribly ugly, but not sure if there's a better way to do
  // it!
  RowVector4d b11(constraint_matrix(4, 12),
                  constraint_matrix(4, 11) - constraint_matrix(5, 12),
                  constraint_matrix(4, 10) - constraint_matrix(5, 11),
                  -constraint_matrix(5, 10));
  RowVector4d b12(constraint_matrix(4, 15),
                  constraint_matrix(4, 14) - constraint_matrix(5, 15),
                  constraint_matrix(4, 13) - constraint_matrix(5, 14),
                  -constraint_matrix(5, 13));
  Matrix<double, 1, 5> b13;
  b13 << constraint_matrix(4, 19),
      constraint_matrix(4, 18) - constraint_matrix(5, 19),
      constraint_matrix(4, 17) - constraint_matrix(5, 18),
      constraint_matrix(4, 16) - constraint_matrix(5, 17),
      -constraint_matrix(5, 16);
  RowVector4d b21(constraint_matrix(6, 12),
                  constraint_matrix(6, 11) - constraint_matrix(7, 12),
                  constraint_matrix(6, 10) - constraint_matrix(7, 11),
                  -constraint_matrix(7, 10));
  RowVector4d b22(constraint_matrix(6, 15),
                  constraint_matrix(6, 14) - constraint_matrix(7, 15),
                  constraint_matrix(6, 13) - constraint_matrix(7, 14),
                  -constraint_matrix(7, 13));
  Matrix<double, 1, 5> b23;
  b23 << constraint_matrix(6, 19),
      constraint_matrix(6, 18) - constraint_matrix(7, 19),
      constraint_matrix(6, 17) - constraint_matrix(7, 18),
      constraint_matrix(6, 16) - constraint_matrix(7, 17),
      -constraint_matrix(7, 16);
  RowVector4d b31(constraint_matrix(8, 12),
                  constraint_matrix(8, 11) - constraint_matrix(9, 12),
                  constraint_matrix(8, 10) - constraint_matrix(9, 11),
                  -constraint_matrix(9, 10));
  RowVector4d b32(constraint_matrix(8, 15),
                  constraint_matrix(8, 14) - constraint_matrix(9, 15),
                  constraint_matrix(8, 13) - constraint_matrix(9, 14),
                  -constraint_matrix(9, 13));
  Matrix<double, 1, 5> b33;
  b33 << constraint_matrix(8, 19),
      constraint_matrix(8, 18) - constraint_matrix(9, 19),
      constraint_matrix(8, 17) - constraint_matrix(9, 18),
      constraint_matrix(8, 16) - constraint_matrix(9, 17),
      -constraint_matrix(9, 16);

  // Eq. 24.
  Matrix<double, 1, 8> p1 = MultiplyPoly(b12, b23) - MultiplyPoly(b13, b22);
  // Eq. 25.
  Matrix<double, 1, 8> p2 = MultiplyPoly(b13, b21) - MultiplyPoly(b11, b23);
  // Eq. 26.
  Matrix<double, 1, 7> p3 = MultiplyPoly(b11, b22) - MultiplyPoly(b12, b21);

  // Eq. 27. Form determinant of B as a 10th degree polynomial.
  Matrix<double, 1, 11> n = MultiplyPoly(p1, b31) + MultiplyPoly(p2, b32) +
                            MultiplyPoly(p3, b33);

  // Step 5. Extract real roots of the 10th degree polynomial.
  Eigen::VectorXd roots;
  FindRealPolynomialRoots(n.transpose().reverse(), &roots);

  essentials->reserve(roots.size());
    rotations->reserve(roots.size());
    translations->reserve(roots.size());

  static const double kTolerance = 1e-12;
  for (int i = 0; i < roots.size(); i++) {
    // We only want non-zero roots
    if (fabs(roots(i)) < kTolerance)
      continue;

    double x = EvaluatePoly(p1, roots(i)) / EvaluatePoly(p3, roots(i));
    double y = EvaluatePoly(p2, roots(i)) / EvaluatePoly(p3, roots(i));
    Matrix<double, 9, 1> temp_sum =
        x * null_space.col(0) + y * null_space.col(1) +
        roots(i) * null_space.col(2) + null_space.col(3);
    // Need to do it like this because temp_sum is a row vector and recasting
    // it as a 3x3 will load it column-major.
    Matrix3d candidate_essential_mat;
    candidate_essential_mat << temp_sum.head<3>().transpose(),
        temp_sum.segment(3, 3).transpose(), temp_sum.tail(3).transpose();
    essentials->push_back(candidate_essential_mat);
      
      Matrix3d rotation_soln;
      Vector3d translation_soln;
      // Decompose into R, t using the first point correspondence.
      DecomposeWithIdealCorrespondence(image1_points[0], image2_points[0],
                                       candidate_essential_mat, &rotation_soln,
                                       &translation_soln);
      rotations->push_back(rotation_soln);
      translations->push_back(translation_soln);
  }
  return (roots.size() > 0);
}

}  // namespace theia
