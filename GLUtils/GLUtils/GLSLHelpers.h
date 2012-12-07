/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLSLHelpers.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <TooN/TooN.h>
#include <TooN/se3.h>

/**
 * @params: fx, fy, cx, cy
 */
TooN::Matrix<4> makeProj( const TooN::Vector<4> &params, double width, double height, double nearPlane = 0.1, double farPlane = 100. );

TooN::Matrix<4> makeScale( const TooN::Vector<3> &scale );
TooN::Matrix<4> makeTranslation( const TooN::Vector<3> &translation );
TooN::Matrix<4> makeRotation( const TooN::SO3<> &rotation );
TooN::Matrix<4> makeModelView( const TooN::SE3<> &pose );
