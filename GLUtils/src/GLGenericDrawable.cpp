/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLGenericDrawable.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/GLGenericDrawable.h>

GLGenericDrawable::GLGenericDrawable() :
	nelem( 0 ), elemSize( 0 ), dataSize( 0 ),
	nattrib( 0 )
{
	data = (GLfloat*) malloc( 0 );

	attribIDs = (GLuint*) malloc( 0 );
	attribSizes = (GLuint*) malloc( 0 );
    
    vboID = 0;
}

void GLGenericDrawable::Create( bool _dynamic )
{
	glGenBuffers( 1, &vboID );

	usage = ( _dynamic ) ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW;
}

GLGenericDrawable::~GLGenericDrawable() {
	free( data );

	free( attribIDs );
	free( attribSizes );

	if ( vboID ) glDeleteBuffers( 1, &vboID );
}

GLfloat* GLGenericDrawable::PushElem() {
	nelem++;
	dataSize += elemSize;
	data = (GLfloat*) realloc( data, dataSize * sizeof(GLfloat) );
	return data + dataSize - elemSize;
}

void GLGenericDrawable::Clear() {
	nelem = 0;
	dataSize = 0;
	data = (GLfloat*) realloc( data, 0 );
}

void GLGenericDrawable::Resize( size_t new_nelem ) {
    nelem = new_nelem;
    dataSize = new_nelem * elemSize;
    data = (GLfloat*) realloc( data, dataSize * sizeof(GLfloat) );
}

void GLGenericDrawable::AddAttrib( GLuint attribID, GLuint attribSize ) {
	nattrib++;

	attribIDs = (GLuint*) realloc( attribIDs, nattrib * sizeof(GLuint) );
	attribSizes = (GLuint*) realloc( attribSizes, nattrib * sizeof(GLuint) );

	attribIDs[nattrib-1] = attribID;
	attribSizes[nattrib-1] = attribSize;


	elemSize += attribSize;
}

//void GLGenericDrawable::AddElem( const Eigen::VectorXf &attrib0 ) {
//	GLfloat *elem = PushElem();
//	for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
//}
//
//void GLGenericDrawable::AddElem( const Eigen::VectorXf &attrib0, const Eigen::VectorXf &attrib1 ) {
//	GLfloat *elem = PushElem();
//	for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
//	for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
//}
//
//void GLGenericDrawable::AddElem( const Eigen::VectorXf &attrib0, const Eigen::VectorXf &attrib1, const Eigen::VectorXf &attrib2 ) {
//	GLfloat *elem = PushElem();
//	for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
//	for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
//	for ( GLuint i = 0; i < attribSizes[2]; i++,elem++ ) *elem = attrib2[i];
//}
//
//void GLGenericDrawable::AddElem( const Eigen::VectorXf &attrib0, const Eigen::VectorXf &attrib1, const Eigen::VectorXf &attrib2, const Eigen::VectorXf &attrib3 ) {
//	GLfloat *elem = PushElem();
//	for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
//	for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
//	for ( GLuint i = 0; i < attribSizes[2]; i++,elem++ ) *elem = attrib2[i];
//	for ( GLuint i = 0; i < attribSizes[3]; i++,elem++ ) *elem = attrib3[i];
//}

void GLGenericDrawable::Commit() {
	glBindBuffer( GL_ARRAY_BUFFER, vboID );
	glBufferData( GL_ARRAY_BUFFER, dataSize * sizeof(GLfloat), data, usage );
    GLfloat *attribPtr = NULL;
	for ( GLuint i = 0; i < nattrib; i++ ) {
		glEnableVertexAttribArray( attribIDs[i] );
		glVertexAttribPointer( attribIDs[i], attribSizes[i], GL_FLOAT, GL_FALSE, elemSize * sizeof( GLfloat ), attribPtr );
		attribPtr += attribSizes[i];
	}
}

void GLGenericDrawable::SetElem( size_t index, const Eigen::VectorXf &attrib0 ) {
	GLfloat *elem = data + elemSize * index;
	for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
}

void GLGenericDrawable::SetElem( size_t index, const Eigen::VectorXf &attrib0, const Eigen::VectorXf &attrib1 ) {
	GLfloat *elem = data + elemSize * index;
	for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
	for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
}

void GLGenericDrawable::SetElem( size_t index, const Eigen::VectorXf &attrib0, const Eigen::VectorXf &attrib1, const Eigen::VectorXf &attrib2 ) {
	GLfloat *elem = data + elemSize * index;
	for ( GLuint i = 0; i < attribSizes[0]; i++,elem++ ) *elem = attrib0[i];
	for ( GLuint i = 0; i < attribSizes[1]; i++,elem++ ) *elem = attrib1[i];
	for ( GLuint i = 0; i < attribSizes[2]; i++,elem++ ) *elem = attrib2[i];
}

void GLGenericDrawable::Update() {
    glBindBuffer( GL_ARRAY_BUFFER, vboID );
    glBufferSubData( GL_ARRAY_BUFFER, 0, dataSize * sizeof(GLfloat), data );
}

void GLGenericDrawable::PushClientState() {
	glBindBuffer( GL_ARRAY_BUFFER, vboID );
    GLfloat *attribPtr = NULL;
	for ( GLuint i = 0; i < nattrib; i++ ) {
		glEnableVertexAttribArray( attribIDs[i] );
		glVertexAttribPointer( attribIDs[i], attribSizes[i], GL_FLOAT, GL_FALSE, elemSize * sizeof( GLfloat ), attribPtr );
		attribPtr += attribSizes[i];
	}
}

void GLGenericDrawable::Draw( GLenum drawType ) {
	glBindBuffer( GL_ARRAY_BUFFER, vboID );
	glDrawArrays( drawType, 0, nelem );
}

void GLGenericDrawable::Draw( GLenum drawType, GLuint index, GLuint length ) {
	glBindBuffer( GL_ARRAY_BUFFER, vboID );
	glDrawArrays( drawType, index, length );
}

void GLGenericDrawable::PopClientState() {
    for ( GLuint i = 0; i < nattrib; i++ ) {
        glDisableVertexAttribArray( attribIDs[i] );
    }
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
}

