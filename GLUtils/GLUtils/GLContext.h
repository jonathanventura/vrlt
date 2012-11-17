/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: GLContext.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef GL_CONTEXT_H
#define GL_CONTEXT_H

#if defined( __APPLE__ )
#include <OpenGL/OpenGL.h>
#endif

#include <cstdlib>

class GLContext
{
protected:
    CGLContextObj save_context;
    CGLContextObj context;
public:
    GLContext();
    ~GLContext();
    
    void create( const GLContext *share = NULL );
    
    void save();
    void restore();
    
    void set();
};

#endif
