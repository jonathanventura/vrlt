/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: CheckGL.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#if defined( __NOKIA__ )
#include <GLES2/gl2.h>
#elif defined( __IPHONE__ )
#include <OpenGLES/ES2/gl.h>
#elif defined( __APPLE__ )
#include <OpenGL/gl.h>
#endif

#include <cstdio>

void checkGL( const char *label )
{
    GLenum err = glGetError();
    if ( err != GL_NO_ERROR )
    {
        fprintf( stderr, "GL error at '%s': %x\n", label, err );
    }
}
