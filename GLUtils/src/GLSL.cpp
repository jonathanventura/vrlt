/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: GLSL.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/GLSL.h>
#include <cstdio>
using namespace std;
using namespace TooN;

GLSLShader::GLSLShader() : mID( 0 )
{
	
}

GLSLShader::~GLSLShader()
{
	if ( mID ) glDeleteShader( mID );
}

bool GLSLShader::LoadFromFile( GLenum type, const char *path )
{
	// open file for reading
	FILE *f = fopen( path, "r" );
	if ( !f ) { fprintf(stderr, "could not open file %s for reading\n", path ); return false; }
	
	// get length of file
	fseek( f, 0, SEEK_END );
	unsigned int length = ftell( f );
	rewind( f );
	
	// allocate buffer (with one extra byte for null termination
	char *buf = new char[ length+1 ];
	buf[length] = '\0';
	
	// read in file
	fread( buf, 1, length, f );
	
	// close file
	fclose( f );
	
	// transfer shader source
	const char *constbuf = buf;
	bool retval = LoadFromSource( type, constbuf );
	
	// delete buffer
	delete [] buf;
	
	return retval;
}

bool GLSLShader::LoadFromSource( GLenum type, const char* buf )
{
	// manage program allocation
	if ( mID ) glDeleteShader( mID );
	mID = glCreateShader( type );
	
	// transfer shader source
	glShaderSource( mID, 1, &buf, NULL );
	
	// compile shader
	glCompileShader( mID );
	
	// check for compilation errors
	GLint compiled = 0;
	glGetShaderiv( mID, GL_COMPILE_STATUS, &compiled );

	if ( compiled != GL_TRUE ) {
		int length, nwritten;
		glGetShaderiv( mID, GL_INFO_LOG_LENGTH, &length );
		
		char *log = new char[length];
		glGetShaderInfoLog( mID, length, &nwritten, log );
		
		fprintf( stderr, "Failed to compile shader: %s\n", log );
		delete [] log;

		return false;
	}
	
	// success
	return true;
}

GLSLProgram::GLSLProgram() : mID( 0 )
{
	
}

GLSLProgram::~GLSLProgram()
{
	if ( mID ) glDeleteProgram( mID );
}

bool GLSLProgram::AttachShaders( const GLSLShader &vertex, const GLSLShader &fragment )
{
	// manage program allocation
	if ( mID ) glDeleteProgram( mID );
	mID = glCreateProgram();
	
    if ( mID == 0 ) return false;
    
	// attach shaders
	glAttachShader( mID, vertex.GetID() );
	glAttachShader( mID, fragment.GetID() );
    
    return true;
}

bool GLSLProgram::Link()
{
    if ( mID == 0 ) return false;
    
	// link program
	glLinkProgram( mID );

	// check for errors
	GLint linked;
	glGetProgramiv( mID, GL_LINK_STATUS, &linked );

	// report errors
	if ( !linked ) {
		int length, nwritten;
		glGetProgramiv( mID, GL_INFO_LOG_LENGTH, &length );
		char* log = new char[length];
		glGetProgramInfoLog( mID, length, &nwritten, log );
		fprintf( stderr, "Failed to link program: %s\n", log );
		delete [] log;
		return false;
	}
	
	// clear dictionary matching uniform names to locations
	mDict.clear();
	
	// get number of active uniforms
	GLint nUniforms;
	glGetProgramiv( mID, GL_ACTIVE_UNIFORMS, &nUniforms );
	
	// get all names and locations of uniforms
	char name[256];
	GLsizei length;
	GLint size;
	GLenum type;
	GLint location;
	for ( GLint i = 0; i < nUniforms; i++ ) {
		glGetActiveUniform( mID, i, 256, &length, &size, &type, name );
        if ( size > 1 ) {
            name[ length-3 ] = '\0';
            for ( GLint j = 0; j < size; j++ ) {
                char myname[256];
                sprintf( myname, "%s[%d]", name, j );
                
                location = glGetUniformLocation( mID, myname );
                mDict[ myname ] = location;
                
                //fprintf( stderr, "key: %s\tvalue: %d\n", myname, location );
            }
        } else {
            location = glGetUniformLocation( mID, name );
            mDict[ name ] = location;
		
            //fprintf( stderr, "key: %s\tvalue: %d\n", name, location );
        }
	}

	return true;
}

void GLSLProgram::Use()
{
	glUseProgram( mID );
}

void GLSLProgram::UnUse()
{
	glUseProgram( 0 );
}

void GLSLProgram::BindAttribute( const string &name, GLenum index )
{
	glBindAttribLocation( mID, index, name.c_str() );

}

//void GLSLProgram::SetUniform( const string &name, GLdouble value )
//{
	//glUniform1f( mDict[ name ], value );
//}

void GLSLProgram::SetUniform( const string &name, GLfloat value )
{
	glUniform1f( mDict[ name ], value );
}

void GLSLProgram::SetUniform( const string &name, GLuint value )
{
	glUniform1i( mDict[ name ], value );
}



void GLSLProgram::SetUniform( const string &name, const Vector<2> &value )
{
	glUniform2f( mDict[ name ], value[0], value[1] );
}

void GLSLProgram::SetUniform( const string &name, const Vector<3> &value )
{
	glUniform3f( mDict[ name ], value[0], value[1], value[2] );
}

void GLSLProgram::SetUniform( const string &name, const Vector<4> &value )
{
	glUniform4f( mDict[ name ], value[0], value[1], value[2], value[3] );
}



void GLSLProgram::SetUniform( const string &name, const Matrix<2> &value )
{
	GLfloat buf[4];
	buf[0] = value(0,0);	buf[1] = value(1,0);
	buf[2] = value(1,0);	buf[3] = value(1,1);
	glUniformMatrix2fv( mDict[ name ], 1, GL_FALSE, buf );
}

void GLSLProgram::SetUniform( const string &name, const Matrix<3> &value )
{
	GLfloat buf[9];
	buf[0] = value(0,0);	buf[1] = value(1,0);	buf[2] = value(2,0);
	buf[3] = value(0,1);	buf[4] = value(1,1);	buf[5] = value(2,1);
	buf[6] = value(0,2);	buf[7] = value(1,2);	buf[8] = value(2,2);
	glUniformMatrix3fv( mDict[ name ], 1, GL_FALSE, buf );
}

void GLSLProgram::SetUniform( const string &name, const Matrix<4> &value )
{
	GLfloat buf[16];
	buf[0] = value(0,0);	buf[1] = value(1,0);	buf[2] = value(2,0);	buf[3] = value(3,0);
	buf[4] = value(0,1);	buf[5] = value(1,1);	buf[6] = value(2,1);	buf[7] = value(3,1);
	buf[8] = value(0,2);	buf[9] = value(1,2);	buf[10]= value(2,2);	buf[11]= value(3,2);
	buf[12]= value(0,3);	buf[13]= value(1,3);	buf[14]= value(2,3);	buf[15]= value(3,3);
	glUniformMatrix4fv( mDict[ name ], 1, GL_FALSE, buf );
}




