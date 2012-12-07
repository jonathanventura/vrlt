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
using namespace TooN;

Matrix<4> makeProj( const Vector<4> &params, double width, double height, double nearPlane, double farPlane )
{
    double left, top, right, bottom;
    
    left = -nearPlane * params[2] / params[0];
    top = nearPlane * params[3] / params[1];
    right = nearPlane * ( width - params[2] ) / params[0];
    bottom = - nearPlane * ( height - params[3] ) / params[1];
    
    Matrix<4> proj = Zeros;
    proj(0,0) = ( 2 * nearPlane ) / ( right - left );
    proj(0,2) = ( right + left ) / ( right - left );
    proj(1,1) = ( 2 * nearPlane ) / ( top - bottom );
    proj(1,2) = ( top + bottom ) / ( top - bottom );
    proj(2,2) = - ( ( farPlane + nearPlane ) / ( farPlane - nearPlane ) );
    proj(2,3) = - ( ( 2 * farPlane * nearPlane ) / ( farPlane - nearPlane ) );
    proj(3,2) = -1;
    
    return proj;
}

TooN::Matrix<4> makeScale( const TooN::Vector<3> &scale )
{
    Matrix<4> mat = Identity;
    mat(0,0) = scale[0];
    mat(1,1) = scale[1];
    mat(2,2) = scale[2];
    return mat;
}

TooN::Matrix<4> makeTranslation( const TooN::Vector<3> &translation )
{
    Matrix<4> mat = Identity;
    mat(0,3) = translation[0];
    mat(1,3) = translation[1];
    mat(2,3) = translation[2];
    return mat;
}

TooN::Matrix<4> makeRotation( const TooN::SO3<> &rotation )
{
    Matrix<4> mat = Identity;
    mat.slice<0,0,3,3>() = rotation.get_matrix();
    return mat;
}

TooN::Matrix<4> makeModelView( const TooN::SE3<> &pose )
{
    Matrix<4> mat = makeTranslation( pose.get_translation() ) * makeRotation( pose.get_rotation() );
    return mat;
}
