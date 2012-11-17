/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: GLContext.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include "GLContext.h"

GLContext::GLContext() : save_context( NULL ), context( NULL )
{
    
}

GLContext::~GLContext()
{
    if ( context != NULL )
    {
        CGLDestroyContext( context );
    }
}

void GLContext::create( const GLContext *share )
{
    if ( context != NULL ) return;
    
    save_context = CGLGetCurrentContext();
    
    CGLPixelFormatAttribute attribs[] = {
        kCGLPFANoRecovery,
        kCGLPFADoubleBuffer,
        kCGLPFAAccelerated,
        kCGLPFAColorSize, (CGLPixelFormatAttribute)24,
        kCGLPFADepthSize, (CGLPixelFormatAttribute)24,
        (CGLPixelFormatAttribute)0
    };
    CGLPixelFormatObj pix;
    GLint npix;
    CGLChoosePixelFormat(attribs, &pix, &npix );
    
    CGLContextObj share_context = NULL;
    if ( share != NULL ) share_context = share->context;
    
    CGLCreateContext( pix, share_context, &context );
}

void GLContext::save()
{
    save_context = CGLGetCurrentContext();
}

void GLContext::restore()
{
    CGLSetCurrentContext( save_context );
}

void GLContext::set()
{
    CGLSetCurrentContext( context );
}
