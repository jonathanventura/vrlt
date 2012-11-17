/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: GLDrawable.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef GLDRAWABLE_H
#define GLDRAWABLE_H

#if defined( __NOKIA__ )
#include <GLES1/gl.h>
#elif defined( __IPHONE__ )
#include <OpenGLES/ES1/gl.h>
#elif defined( __APPLE__ )
#include <OpenGL/gl.h>
#endif

#include <TooN/TooN.h>

#include <cstdio>

/** GLDrawable
 *
 * A drawable object which uses a vertex buffer object and named attributes.
 *
 */
class GLDrawable {
protected:
	GLfloat *data;
    GLuint texCoordSize;
    GLuint colorSize;
    GLuint vertexSize;
    GLuint format;
    
	GLuint nelem;
	GLuint elemSize;
	GLuint dataSize;
	
	GLuint vboID;
	
	GLenum usage;
	
	GLfloat* PushElem();
public:
	/** Constructor
     * @param _format the format for glInterleavedArrays()
	 * @param _dynamic set true to use GL_DYNAMIC vertex buffer storage, if you will be changing the data repeatedly
	 */
	GLDrawable( GLenum _format, bool _dynamic = false );
	~GLDrawable();
	
	/** Clear the buffer */
	void Clear();
    /** Resize the data */
    void Resize( GLuint new_nelem );
	/** Add an element */
	void AddElem( const TooN::Vector<> &texture, const TooN::Vector<> &color, const TooN::Vector<> &vertex );
	/** Set an element */
	void SetElem( GLuint index, const TooN::Vector<> &texture, const TooN::Vector<> &color, const TooN::Vector<> &vertex );
	/** Copy the data to the vertex buffer object
	 * This must be called after adding elements and before drawing.
	 */
	void Commit();
	/** Update the data in the vertex buffer object
	 * This must be called after an initial commit and before drawing.
	 */
    void Update( GLuint index, GLuint length );
    /** Enable relevant arrays 
     * This must be called before drawing
     */
	void Draw( GLenum drawType );
    /** Draw subset
     */
    void Draw( GLenum drawType, GLuint index, GLuint length );
};

#endif
