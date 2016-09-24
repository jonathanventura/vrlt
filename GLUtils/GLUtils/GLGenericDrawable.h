/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLGenericDrawable.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef GLDRAWABLE_H
#define GLDRAWABLE_H

#if defined( __NOKIA__) || defined ( __linux__ )
#include <GLES2/gl2.h>
#elif defined( __IPHONE__ )
#include <OpenGLES/ES2/gl.h>
#elif defined( __APPLE__ )
#include <OpenGL/gl.h>
#endif

#include <Eigen/Core>

#include <cstdio>

/** GLGenericDrawable
 *
 * A drawable object which uses a vertex buffer object and generic attributes.
 *
 */
class GLGenericDrawable {
protected:
	GLfloat *data;
	GLuint nelem;
	GLuint elemSize;
	GLuint dataSize;

	GLuint *attribIDs;
	GLuint *attribSizes;
	GLuint nattrib;

	GLuint vboID;

	GLenum usage;

	GLfloat* PushElem();
public:
	GLGenericDrawable();
	~GLGenericDrawable();

	/** Create the buffer object
	 * @param _dynamic set true to use GL_DYNAMIC vertex buffer storage, if you will be changing the data repeatedly
	 */
    void Create( bool _dynamic = false );
    
	/** Clear the buffer */
	void Clear();
    /** Change the number of elements in the buffer */
    void Resize( size_t new_nelem );
	/** Add an attribute
	 * @param attribID the attribute index
	 * @param attribSize the number of components in the attribute (1,2,3 or 4)
	 */
	void AddAttrib( GLuint attribID, GLuint attribSize );
	/** Add an element with one attribute */
	template <typename T0> void AddElem( const Eigen::MatrixBase<T0> &attrib0 )
    {
        GLfloat *elem = PushElem();
        for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
    }
	/** Add an element with two attributes */
	template <typename T0, typename T1> void AddElem( const Eigen::MatrixBase<T0> &attrib0, const Eigen::MatrixBase<T1> &attrib1 )
    {
        GLfloat *elem = PushElem();
        for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
        for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
    }
	/** Add an element with three attributes */
	template <typename T0, typename T1, typename T2> void AddElem( const Eigen::MatrixBase<T0> &attrib0, const Eigen::MatrixBase<T1> &attrib1, const Eigen::MatrixBase<T2> &attrib2 )
    {
        GLfloat *elem = PushElem();
        for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
        for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
        for ( GLuint i = 0; i < attribSizes[2]; i++,elem++ ) *elem = attrib2[i];
    }
	/** Add an element with four attributes */
	template <typename T0, typename T1, typename T2, typename T3> void AddElem( const Eigen::MatrixBase<T0> &attrib0, const Eigen::MatrixBase<T1> &attrib1, const Eigen::MatrixBase<T2> &attrib2, const Eigen::MatrixBase<T3> &attrib3 )
    {
        GLfloat *elem = PushElem();
        for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
        for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
        for ( GLuint i = 0; i < attribSizes[2]; i++,elem++ ) *elem = attrib2[i];
        for ( GLuint i = 0; i < attribSizes[3]; i++,elem++ ) *elem = attrib3[i];
    }
	/** Copy the data to the vertex buffer object
	 * This must be called after adding elements and before drawing.
	 */
	void Commit();
    /** Update the data in a specific element with one attribute */
    void SetElem( size_t index, const Eigen::VectorXf &attrib0 );
    /** Update the data in a specific element with two attributes */
    void SetElem( size_t index, const Eigen::VectorXf &attrib0, const Eigen::VectorXf &attrib1 );
    /** Update the data in a specific element with three attributes */
    void SetElem( size_t index, const Eigen::VectorXf &attrib0, const Eigen::VectorXf &attrib1, const Eigen::VectorXf &attrib2 );
    /** Update the data in the vertex buffer object
     * This must be called after setting elements and before drawing.
     */
    void Update();
    /** Push client state
     */
    void PushClientState();
	/** Draw
	 * @param drawType the draw command to use (e.g. GL_TRIANGLE_STRIPS)
	 */
	void Draw( GLenum drawType );
    /** Draw
	 * @param drawType the draw command to use (e.g. GL_TRIANGLE_STRIPS)
	 */
	void Draw( GLenum drawType, GLuint index, GLuint length );
    /** Pop client state
     */
    void PopClientState();
};

#endif
