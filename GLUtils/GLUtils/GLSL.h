/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: GLSL.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef GLSL_H
#define GLSL_H

#if defined( __NOKIA__ )
#include <GLES2/gl2.h>
#elif defined( __IPHONE__ )
#include <OpenGLES/ES2/gl.h>
#elif defined( __APPLE__ )
#include <OpenGL/gl.h>
#endif

#include <TooN/TooN.h>

#include <map>
#include <string>

/** GLSL Shader
 * Encapsulates GLSL vertex and fragment shaders.
 */
class GLSLShader {
	GLuint mID;
public:
	/** Create the shader.
	 */
	GLSLShader();
	~GLSLShader();
	
	/** Get the shader index */
	GLuint GetID() const { return mID; }
	/** Load the shader source from a file.
	 * @param type the shader type: GL_FRAGMENT or GL_VERTEX
	 * @param path path to the file
	 */
	bool LoadFromFile( GLenum type, const char *path );
	/** Load the shader source from a string.
	 * @param type the shader type: GL_FRAGMENT or GL_VERTEX
	 * @param buf the string containing the source
	 */
	bool LoadFromSource( GLenum type, const char *buf );
};

/** GLSL Program
 * Encapsulates a GLSL shader program.
 * This maintains a map of variable names,
 * so you can directly set uniform variables by name.
 * NB: The program must be in use before uniforms can be set.
 */
class GLSLProgram {
	GLuint mID;
	std::map<std::string,GLint> mDict;
public:
	/** Create the program.
	 */
	GLSLProgram();
	~GLSLProgram();
	
	/** Attach the shaders.
	 * @param vertex the vertex shader
	 * @param fragment the fragment shader
	 */
	bool AttachShaders( const GLSLShader &vertex, const GLSLShader &fragment );
	
    /* Link the program.
     */
    bool Link();
    
	/** Enable the program. */
	void Use();
	/** Disable the program.
	 * This is static since it disables all shader programs.
	 */
	static void UnUse();
	
	/** Bind an attribute name to an index
	 * @param name the attribute name in the shader.
	 * @param index the attribute index
	 */
	void BindAttribute( const std::string &name, GLenum index );
	
	void SetUniform( const std::string &name, GLuint value );
	void SetUniform( const std::string &name, GLfloat value );
	void SetUniform( const std::string &name, const TooN::Vector<2> &value );
	void SetUniform( const std::string &name, const TooN::Vector<3> &value );
	void SetUniform( const std::string &name, const TooN::Vector<4> &value );
	void SetUniform( const std::string &name, const TooN::Matrix<2> &value );
	void SetUniform( const std::string &name, const TooN::Matrix<3> &value );
	void SetUniform( const std::string &name, const TooN::Matrix<4> &value );
};

#endif
