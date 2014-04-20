/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLShadow.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef GL_SHADOW_H
#define GL_SHADOW_H

#if defined( __NOKIA__ )
#include <GLES2/gl2.h>
#elif defined( __IPHONE__ )
#include <OpenGLES/ES2/gl.h>
#elif defined( __APPLE__ )
#include <OpenGL/gl.h>
#endif

#include "GLSL.h"

class GLShadowShader
{
public:
    GLSLShader vertexShader;
    GLSLShader fragmentShader;
    GLSLProgram shaderProgram;
    
    void Create( bool rgb = false );
    void SetModelViewProj( const Eigen::Matrix4d &mvp );
    void SetLightDirection( const Eigen::Vector3d &lightDirection );
    void SetPlane( const Eigen::Vector4d &plane );
    void SetColor( const Eigen::Vector4d &color );
};

#endif