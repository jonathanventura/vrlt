/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLModel.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef GL_MODEL_H
#define GL_MODEL_H

#if defined( __NOKIA__ )
#include <GLES2/gl2.h>
#elif defined( __IPHONE__ )
#include <OpenGLES/ES2/gl.h>
#elif defined( __APPLE__ )
#include <OpenGL/gl.h>
#endif

#include <cstdlib>
#include <vector>

#include "GLGenericDrawable.h"
#include "GLSL.h"

class GLModelShader
{
public:
    GLSLShader vertexShader;
    GLSLShader fragmentShader;
    GLSLProgram shaderProgram;
    
    void Create();
    void SetAmbient( float ambient );
    void SetLightDirection( const TooN::Vector<3> &lightDirection );
    void SetModelViewProj( const TooN::Matrix<4> &mvp );
};

struct GLModelObject
{
    GLuint texID;
    GLuint index;
    GLuint length;
    GLenum type;
};

class GLModel
{
public:
    // this handles deleting objects and GL objects
    ~GLModel();
    
    // put all vertex data in here
    GLGenericDrawable *drawable;
    
    std::vector<GLuint> textures;
    std::vector<GLModelObject> objects;
    
    // sort by texture, to optimize rendering
    void SortObjectsByTexture();
    
    void Render();
    void RenderNoTextureBind();
};

#endif
