/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: GLSLPrograms.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/GLSLPrograms.h>

static const char *imagevertexsource =
"attribute vec2 a_position;\n"
"attribute vec2 a_texCoord;\n"
"varying vec2 v_texCoord;\n"
"void main() {\n"
"   gl_Position = vec4( a_position, 0, 1 );\n"
"   v_texCoord = a_texCoord;\n"
"}\n";

static const char *imagefragmentsource =
#if defined( __IPHONE__ )
"precision highp float;\n"
#endif
"varying vec2 v_texCoord;\n"
"uniform sampler2D u_texture;\n"
"void main() {\n"
"   gl_FragColor = texture2D( u_texture, v_texCoord );\n"
"}\n";

void GLImageShader::Create()
{
    vertexShader.LoadFromSource( GL_VERTEX_SHADER, imagevertexsource );
    fragmentShader.LoadFromSource( GL_FRAGMENT_SHADER, imagefragmentsource );
    shaderProgram.AttachShaders( vertexShader, fragmentShader );
    
    shaderProgram.BindAttribute( "a_position", 0 );
    shaderProgram.BindAttribute( "a_texCoord", 1 );
    shaderProgram.Link();

    shaderProgram.Use();
    SetTextureUnit( 0 );
    shaderProgram.UnUse();
    
}

void GLImageShader::SetTextureUnit( const GLuint index )
{
    shaderProgram.SetUniform( "u_texture", index );
}

static const char *yuvimagevertexsource =
"attribute vec2 a_position;\n"
"attribute vec2 a_texCoord;\n"
"varying vec2 v_texCoord;\n"
"void main() {\n"
"   gl_Position = vec4( a_position, 0, 1 );\n"
"   v_texCoord = a_texCoord;\n"
"}\n";

static const char *yuvimagefragmentsource =
#if defined( __IPHONE__ )
"precision highp float;\n"
#endif
"varying vec2 v_texCoord;\n"
"uniform sampler2D u_textureY;\n"
"uniform sampler2D u_textureU;\n"
"uniform sampler2D u_textureV;\n"
"uniform mat3 u_colorTransform;\n"
"void main() {\n"
"   gl_FragColor.r = texture2D( u_textureY, v_texCoord ).r;\n"
"   gl_FragColor.g = texture2D( u_textureU, v_texCoord ).r;\n"
"   gl_FragColor.b = texture2D( u_textureV, v_texCoord ).r;\n"
"   gl_FragColor.a = 1.;\n"
"   gl_FragColor.rgb = u_colorTransform * gl_FragColor.rgb;\n"
"}\n";

void GLYUVImageShader::Create()
{
    vertexShader.LoadFromSource( GL_VERTEX_SHADER, yuvimagevertexsource );
    fragmentShader.LoadFromSource( GL_FRAGMENT_SHADER, yuvimagefragmentsource );
    shaderProgram.AttachShaders( vertexShader, fragmentShader );
    
    shaderProgram.BindAttribute( "a_position", 0 );
    shaderProgram.BindAttribute( "a_texCoord", 1 );
    shaderProgram.Link();

    shaderProgram.Use();
    shaderProgram.SetUniform( "u_textureY" , (GLuint)0 );
    shaderProgram.SetUniform( "u_textureU" , (GLuint)1 );
    shaderProgram.SetUniform( "u_textureV" , (GLuint)2 );
    TooN::Matrix<3> transform;
    transform[0] = TooN::makeVector( 1, 0, 1.13983 );
    transform[1] = TooN::makeVector( 1, -0.39465, -0.58060 );
    transform[2] = TooN::makeVector( 1, 2.03211, 0 );
    shaderProgram.SetUniform( "u_colorTransform", transform );
    shaderProgram.UnUse();
    
}

static const char *ycbcrimagevertexsource =
"attribute vec2 a_position;\n"
"attribute vec2 a_texCoord;\n"
"varying vec2 v_texCoord;\n"
"void main() {\n"
"   gl_Position = vec4( a_position, 0, 1 );\n"
"   v_texCoord = a_texCoord;\n"
"}\n";

static const char *ycbcrimagefragmentsource =
#if defined( __IPHONE__ )
"precision highp float;\n"
#endif
"varying vec2 v_texCoord;\n"
"uniform sampler2D u_textureY;\n"
"uniform sampler2D u_textureCbCr;\n"
"void main() {\n"
"   float Y = texture2D( u_textureY, v_texCoord ).r;\n"
"   vec2 CbCr = texture2D( u_textureCbCr, v_texCoord ).ra;\n"
"   Y = Y - 16./255.;\n"
"   float Cb = CbCr.r - 128./255.;\n"
"   float Cr = CbCr.g - 128./255.;\n"
"   gl_FragColor.r = 1.164 * Y + 1.596 * Cr;\n"
"   gl_FragColor.g = 1.164 * Y - 0.813 * Cr - 0.392 * Cb;\n"
"   gl_FragColor.b = 1.164 * Y + 2.017 * Cb;\n"
"   gl_FragColor.a = 1.;\n"
"}\n";

void GLYCbCrImageShader::Create()
{
    vertexShader.LoadFromSource( GL_VERTEX_SHADER, ycbcrimagevertexsource );
    fragmentShader.LoadFromSource( GL_FRAGMENT_SHADER, ycbcrimagefragmentsource );
    shaderProgram.AttachShaders( vertexShader, fragmentShader );
    
    shaderProgram.BindAttribute( "a_position", 0 );
    shaderProgram.BindAttribute( "a_texCoord", 1 );
    shaderProgram.Link();

    shaderProgram.Use();
    shaderProgram.SetUniform( "u_textureY" , (GLuint)0 );
    shaderProgram.SetUniform( "u_textureCbCr" , (GLuint)1 );
    shaderProgram.UnUse();
    
}



static const char *pointvertexsource =
"attribute vec3 a_position;\n"
"uniform mat4 u_modelViewProj;\n"
"void main() {\n"
"   gl_PointSize = 5.0;\n"
"   gl_Position = u_modelViewProj * vec4( a_position, 1 );\n"
"}\n";

static const char *pointfragmentsource =
"#ifdef GL_ES\n"
"precision highp float;\n"
"#endif\n"
"uniform vec3 u_color;\n"
"void main() {\n"
"   gl_FragColor = vec4( u_color, 1 );\n"
"}\n";

void GLPointShader::Create()
{
    vertexShader.LoadFromSource( GL_VERTEX_SHADER, pointvertexsource );
    fragmentShader.LoadFromSource( GL_FRAGMENT_SHADER, pointfragmentsource );
    shaderProgram.AttachShaders( vertexShader, fragmentShader );
    
    shaderProgram.BindAttribute( "a_position", 0 );
    
    shaderProgram.Link();
}

void GLPointShader::SetModelViewProj( const TooN::Matrix<4> &modelViewProj )
{
    shaderProgram.SetUniform( "u_modelViewProj", modelViewProj );
}

void GLPointShader::SetColor( const TooN::Vector<3> &color )
{
    shaderProgram.SetUniform( "u_color", color );
}
