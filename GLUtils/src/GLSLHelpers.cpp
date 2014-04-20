/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLSLHelpers.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/GLSLHelpers.h>

Eigen::Matrix4d makeProj( const Eigen::Vector4d &params, double width, double height, double nearPlane, double farPlane )
{
    double left, top, right, bottom;
    
    left = -nearPlane * params[2] / params[0];
    top = nearPlane * params[3] / params[1];
    right = nearPlane * ( width - params[2] ) / params[0];
    bottom = - nearPlane * ( height - params[3] ) / params[1];
    
    Eigen::Matrix4d proj = Eigen::Matrix4d::Zero();
    proj(0,0) = ( 2 * nearPlane ) / ( right - left );
    proj(0,2) = ( right + left ) / ( right - left );
    proj(1,1) = ( 2 * nearPlane ) / ( top - bottom );
    proj(1,2) = ( top + bottom ) / ( top - bottom );
    proj(2,2) = - ( ( farPlane + nearPlane ) / ( farPlane - nearPlane ) );
    proj(2,3) = - ( ( 2 * farPlane * nearPlane ) / ( farPlane - nearPlane ) );
    proj(3,2) = -1;
    
    return proj;
}

Eigen::Matrix4d makeScale( const Eigen::Vector3d &scale )
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat(0,0) = scale[0];
    mat(1,1) = scale[1];
    mat(2,2) = scale[2];
    return mat;
}

Eigen::Matrix4d makeTranslation( const Eigen::Vector3d &translation )
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat(0,3) = translation[0];
    mat(1,3) = translation[1];
    mat(2,3) = translation[2];
    return mat;
}

Eigen::Matrix4d makeRotation( const Eigen::Matrix3d &rotation )
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3,3>(0,0) = rotation.matrix();
    return mat;
}

Eigen::Matrix4d makeModelView( const Sophus::SE3d &pose )
{
    Eigen::Matrix4d mat = makeTranslation( pose.translation() ) * makeRotation( pose.so3() );
    return mat;
}
