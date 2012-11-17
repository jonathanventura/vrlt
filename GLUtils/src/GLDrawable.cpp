/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: GLDrawable.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include "GLDrawable.h"

#include <cstdio>

GLDrawable::GLDrawable( GLenum _format, bool _dynamic ) :
    nelem( 0 ),
    dataSize( 0 ),
    format( _format )
{
	data = (GLfloat*) malloc( 0 );
	
    texCoordSize = colorSize = vertexSize = 0;
    
    switch ( format ) {
        case GL_T2F_V3F:
            texCoordSize = 2;
            vertexSize = 3;
            break;
        
        default:
            fprintf( stderr, "warning: unsupported format for GLDrawable\n" );
            break;
    }
    elemSize = vertexSize + colorSize + texCoordSize;

    glGenBuffers( 1, &vboID );
	
	usage = ( _dynamic ) ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW;
}

GLDrawable::~GLDrawable() {
	free( data );
	
	glDeleteBuffers( 1, &vboID );
}

GLfloat* GLDrawable::PushElem() {
	nelem++;
	dataSize += elemSize;
	data = (GLfloat*) realloc( data, dataSize * sizeof(GLfloat) );
	return data + dataSize - elemSize;
}

void GLDrawable::Clear() {
	data = (GLfloat*) realloc( data, 0 );
	nelem = 0;
	dataSize = 0;
}

void GLDrawable::Resize( GLuint new_nelem ) {
    nelem = new_nelem;
    dataSize = nelem * elemSize;
    data = (GLfloat*) realloc( data, dataSize * sizeof(GLfloat) );
}

void GLDrawable::AddElem( const TooN::Vector<> &texture, const TooN::Vector<> &color, const TooN::Vector<> &vertex ) {
	GLfloat *elem = PushElem();
	for ( GLuint i = 0; i < texCoordSize; i++,elem++ ) *elem = texture[i];
	for ( GLuint i = 0; i < colorSize; i++,elem++ ) *elem = color[i];
	for ( GLuint i = 0; i < vertexSize; i++,elem++ ) *elem = vertex[i];
}

void GLDrawable::SetElem( GLuint index, const TooN::Vector<> &texture, const TooN::Vector<> &color, const TooN::Vector<> &vertex ) {
	GLfloat *elem = data + elemSize * index;
	for ( GLuint i = 0; i < texCoordSize; i++,elem++ ) *elem = texture[i];
	for ( GLuint i = 0; i < colorSize; i++,elem++ ) *elem = color[i];
	for ( GLuint i = 0; i < vertexSize; i++,elem++ ) *elem = vertex[i];
}

void GLDrawable::Commit() {
	glBindBuffer( GL_ARRAY_BUFFER, vboID );
	glBufferData( GL_ARRAY_BUFFER, dataSize * sizeof(GLfloat), data, usage );
    glInterleavedArrays( format, 0, 0 );
}

void GLDrawable::Update( GLuint index, GLuint length ) {
	glBindBuffer( GL_ARRAY_BUFFER, vboID );
	glBufferSubData( GL_ARRAY_BUFFER, index * elemSize * sizeof(GLfloat), length * elemSize * sizeof(GLfloat), data );
    glInterleavedArrays( format, 0, 0 );
}

void GLDrawable::Draw( GLenum drawType ) {
	glBindBuffer( GL_ARRAY_BUFFER, vboID );
	glDrawArrays( drawType, 0, nelem );
}

void GLDrawable::Draw( GLenum drawType, GLuint index, GLuint length )
{
    glBindBuffer( GL_ARRAY_BUFFER, vboID );
	glDrawArrays( drawType, index, length );
}

