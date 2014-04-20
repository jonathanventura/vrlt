/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLSLPrograms.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include "GLSL.h"

/**
 * Renders a texture.
 */
class GLImageShader
{
public:
    GLSLShader vertexShader;
    GLSLShader fragmentShader;
    GLSLProgram shaderProgram;
    
    void Create();
    
    void SetColorTransform( const Eigen::Matrix3d &transform );
    void SetTextureUnit( const GLuint index );
};

/**
 * Renders a YCbCr texture.
 */
class GLYCbCrImageShader
{
public:
    GLSLShader vertexShader;
    GLSLShader fragmentShader;
    GLSLProgram shaderProgram;
    
    void Create();
};

/**
 * Renders a YUV texture.
 */
class GLYUVImageShader
{
public:
    GLSLShader vertexShader;
    GLSLShader fragmentShader;
    GLSLProgram shaderProgram;
    
    void Create();
};


/**
 * Renders 3D points with a single color and modelview projection matrix.
 */
class GLPointShader
{
public:
    GLSLShader vertexShader;
    GLSLShader fragmentShader;
    GLSLProgram shaderProgram;
    
    void Create();
    
    void SetModelViewProj( const Eigen::Matrix4d &modelViewProj );
    void SetColor( const Eigen::Vector3d &color );
};

