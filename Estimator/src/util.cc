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

#include "util.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace theia {

using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

void ComposeProjectionMatrix(const double focal_length[2],
                             const double principle_point[2],
                             const double rotation[9],
                             const double translation[3],
                             double projection_matrix[12]) {
  Matrix3d camera_matrix;
  camera_matrix << focal_length[0], 0.0, principle_point[0],
      0.0, focal_length[1], principle_point[1],
      0.0, 0.0, 1.0;

  Map<Matrix<double, 3, 4> > proj_mat(projection_matrix);
  proj_mat.block<3, 3>(0, 0) = Map<const Matrix3d>(rotation);
  proj_mat.block<3, 1>(0, 3) = Map<const Vector3d>(translation);
}

// For an E or F that is defined such that y^t * E * x = 0
double SampsonDistance(const Matrix3d& F, const Vector2d& x,
                       const Vector2d& y) {
  const Vector3d epiline_x = F * x.homogeneous();
  const Vector3d epiline_y = F.transpose() * y.homogeneous();

  const double numerator_sqrt = y.homogeneous().dot(epiline_x);
  const double denominator = epiline_x.hnormalized().squaredNorm() +
                             epiline_y.hnormalized().squaredNorm();

  // Finally, return the complete Sampson distance.
  return numerator_sqrt * numerator_sqrt / denominator;
}

Eigen::Matrix3d CrossProductMatrix(const Vector3d& cross_vec) {
  Matrix3d cross;
  cross << 0.0, -cross_vec.z(), cross_vec.y(),
      cross_vec.z(), 0.0, -cross_vec.x(),
      -cross_vec.y(), cross_vec.x(), 0.0;
  return cross;
}

// Computes the normalization matrix transformation that centers image points
// around the origin with an average distance of sqrt(2) to the centroid.
// Returns the transformation matrix and the transformed points. This assumes
// that no points are at infinity.
bool NormalizeImagePoints(
    const std::vector<Vector2d>& image_points,
    std::vector<Vector2d>* normalized_image_points,
    Matrix3d* normalization_matrix) {
  Eigen::Map<const Matrix<double, 2, Eigen::Dynamic> > image_points_mat(
      image_points[0].data(), 2, image_points.size());

  // Allocate the output vector and map an Eigen object to the underlying data
  // for efficient calculations.
  normalized_image_points->clear();
  normalized_image_points->resize(image_points.size());
  Eigen::Map<Matrix<double, 2, Eigen::Dynamic> >
      normalized_image_points_mat((*normalized_image_points)[0].data(), 2,
                                  image_points.size());

  // Compute centroid.
  const Vector2d centroid(image_points_mat.rowwise().mean());

  // Calculate average distance to centroid.
  const double mean_dist = (image_points_mat.colwise() - centroid).norm();

  // Create normalization matrix.
  const double norm_factor = sqrt(2.0) / mean_dist;
  *normalization_matrix << norm_factor, 0, -1.0 * norm_factor* centroid.x(),
      0, norm_factor, -1.0 * norm_factor * centroid.y(),
      0, 0, 1;

  // Normalize image points.
  const Matrix<double, 3, Eigen::Dynamic> normalized_homog_points =
      (*normalization_matrix) * image_points_mat.colwise().homogeneous();
  normalized_image_points_mat = normalized_homog_points.colwise().hnormalized();

  return true;
}

}  // namespace theia
