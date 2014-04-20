/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLShadow.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/GLShadow.h>

static const char *vertexsource =
"attribute vec3 a_position;\n"
"attribute vec3 a_normal;\n"
"uniform mat4 u_mvp;\n"
"uniform vec3 u_lightDirection;\n"
"uniform vec4 u_plane;\n"
"varying float v_t;\n"
"varying vec3 v_normal;\n"
"void main() {\n"
"   vec3 v = a_position;\n"
"   vec3 r = u_lightDirection;\n"
"   vec3 N = u_plane.xyz;\n"
"   float Nv = dot( N, v );\n"
"   float Nr = dot( N, r );\n"
"   float D = u_plane.w;\n"
"   float t = ( - D - Nv ) / Nr;\n"
"   vec3 p = v + t * r;\n"
"   gl_Position = u_mvp * vec4( p, 1 );\n"
"   v_normal = a_normal;\n"
"}\n";

static const char *rgbfragmentsource =
"#ifdef GL_ES\n"
"precision highp float;\n"
"#endif\n"
"varying vec3 v_normal;\n"
"uniform vec3 u_lightDirection;\n"
"uniform vec4 u_color;\n"
"uniform sampler2D u_textureRGB;\n"
"void main() {\n"
"   if ( dot( v_normal, u_lightDirection ) > 0. ) discard;\n"
"   gl_FragColor = u_color;\n"
//"   vec2 v_texCoord = vec2( gl_FragCoord.x / 1280., 1. - gl_FragCoord.y / 720. ); \n"
//"   gl_FragColor = texture2D( u_textureRGB, v_texCoord );\n"
//"   gl_FragColor.rgb = ( 1. - u_color.a ) * gl_FragColor.rgb;\n"
"}\n";

static const char *ycbcrfragmentsource =
"#ifdef GL_ES\n"
"precision highp float;\n"
"#endif\n"
"varying vec3 v_normal;\n"
"uniform vec3 u_lightDirection;\n"
"uniform vec4 u_color;\n"
"uniform sampler2D u_textureY;\n"
"uniform sampler2D u_textureCbCr;\n"
"void main() {\n"
"   if ( dot( v_normal, u_lightDirection ) > 0. ) discard;\n"
"   vec2 v_texCoord = vec2( gl_FragCoord.x / 1024., 1. - gl_FragCoord.y / 748. ); \n"
"   float Y = texture2D( u_textureY, v_texCoord ).r;\n"
"   vec2 CbCr = texture2D( u_textureCbCr, v_texCoord ).ra;\n"
"   Y = Y - 16./255.;\n"
"   float Cb = CbCr.r - 128./255.;\n"
"   float Cr = CbCr.g - 128./255.;\n"
"   gl_FragColor.r = 1.164 * Y + 1.596 * Cr;\n"
"   gl_FragColor.g = 1.164 * Y - 0.813 * Cr - 0.392 * Cb;\n"
"   gl_FragColor.b = 1.164 * Y + 2.017 * Cb;\n"
"   gl_FragColor.rgb = ( 1. - u_color.a ) * gl_FragColor.rgb;\n"
"   gl_FragColor.a = 1.;\n"
"}\n";

void GLShadowShader::Create( bool rgb )
{
    vertexShader.LoadFromSource( GL_VERTEX_SHADER, vertexsource );
    if ( rgb ) {
        fragmentShader.LoadFromSource( GL_FRAGMENT_SHADER, rgbfragmentsource );
    } else {
        fragmentShader.LoadFromSource( GL_FRAGMENT_SHADER, ycbcrfragmentsource );
    }
    shaderProgram.AttachShaders( vertexShader, fragmentShader );
    
    shaderProgram.BindAttribute( "a_position", 0 );
    shaderProgram.BindAttribute( "a_normal", 3 );
    
    shaderProgram.Link();
    
    shaderProgram.Use();
    
    Eigen::Vector3d lightDirection;
    lightDirection << 1, 1, -1;
    lightDirection.normalize();
    shaderProgram.SetUniform( "u_lightDirection", lightDirection );
    
    Eigen::Vector4d plane;
    plane << 0, -1, 0, 0;
    shaderProgram.SetUniform( "u_plane", plane );
    
    Eigen::Vector4d color;
    color << 0, 0, 0, 0.5;
    shaderProgram.SetUniform( "u_color", color );
    
    if ( rgb ) {
        shaderProgram.SetUniform( "u_textureRGB" , (GLuint)0 );
    } else {
        shaderProgram.SetUniform( "u_textureY" , (GLuint)0 );
        shaderProgram.SetUniform( "u_textureCbCr" , (GLuint)1 );
    }
    
    shaderProgram.UnUse();
}

void GLShadowShader::SetLightDirection( const Eigen::Vector3d &lightDirection )
{
    shaderProgram.SetUniform( "u_lightDirection", lightDirection );
}

void GLShadowShader::SetModelViewProj( const Eigen::Matrix4d &mvp )
{
    shaderProgram.SetUniform( "u_mvp", mvp );
}

void GLShadowShader::SetPlane( const Eigen::Vector4d &plane )
{
    shaderProgram.SetUniform( "u_plane", plane );
}

void GLShadowShader::SetColor( const Eigen::Vector4d &color )
{
    shaderProgram.SetUniform( "u_color", color );
}
