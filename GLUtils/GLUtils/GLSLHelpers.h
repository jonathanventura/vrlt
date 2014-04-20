/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLSLHelpers.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <Eigen/Core>
#include <Sophus/se3.hpp>

/**
 * @params: fx, fy, cx, cy
 */
Eigen::Matrix4d makeProj( const Eigen::Vector4d &params, double width, double height, double nearPlane = 0.1, double farPlane = 100. );

Eigen::Matrix4d makeScale( const Eigen::Vector3d &scale );
Eigen::Matrix4d makeTranslation( const Eigen::Vector3d &translation );
Eigen::Matrix4d makeRotation( const Sophus::SO3d &rotation );
Eigen::Matrix4d makeModelView( const Sophus::SE3d &pose );
