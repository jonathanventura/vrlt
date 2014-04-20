/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: GLModel.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/GLModel.h>
#include <GLUtils/CheckGL.h>

#include <cstdio>
#include <iostream>
#include <algorithm>
using namespace std;

static const char *vertexsource =
"attribute vec3 a_position;\n"
"attribute vec2 a_texCoord;\n"
"attribute vec4 a_scaleOffset;\n"
"attribute vec3 a_normal;\n"
"uniform mat4 u_mvp;\n"
"varying vec2 v_texCoord;\n"
"varying vec4 v_scaleOffset;\n"
"varying vec3 v_normal;\n"
"void main() {\n"
"   gl_Position = u_mvp * vec4( a_position, 1 );\n"
"   v_texCoord = a_texCoord;\n"
"   v_scaleOffset = a_scaleOffset;\n"
"   v_normal = a_normal;\n"
"}\n";

static const char *fragmentsource =
"#ifdef GL_ES\n"
"precision highp float;\n"
"#endif\n"
"varying vec2 v_texCoord;\n"
"varying vec4 v_scaleOffset;\n"
"varying vec3 v_normal;\n"
"uniform sampler2D u_texture;\n"
"uniform float u_ambient;\n"
"uniform vec3 u_lightDirection;\n"
"void main() {\n"
"   vec4 texcolor = texture2D( u_texture, v_scaleOffset.xy * fract( v_texCoord ) + v_scaleOffset.zw );\n"
"   float diffuse = max( dot( v_normal, -u_lightDirection ), 0. );\n"
"   gl_FragColor = texcolor * ( u_ambient + diffuse );\n"
"}\n";

void GLModelShader::Create()
{
    vertexShader.LoadFromSource( GL_VERTEX_SHADER, vertexsource );
    fragmentShader.LoadFromSource( GL_FRAGMENT_SHADER, fragmentsource );
    shaderProgram.AttachShaders( vertexShader, fragmentShader );
    
    shaderProgram.BindAttribute( "a_position", 0 );
    shaderProgram.BindAttribute( "a_texCoord", 1 );
    shaderProgram.BindAttribute( "a_scaleOffset", 2 );
    shaderProgram.BindAttribute( "a_normal", 3 );
    
    shaderProgram.Link();
    
    shaderProgram.Use();
    
    shaderProgram.SetUniform( "u_texture", (GLuint)0 );
    
    Eigen::Vector3d lightDirection;
//    lightDirection << 1, 1, -1;
    lightDirection << 1, 1, 1;
    lightDirection.normalize();
    shaderProgram.SetUniform( "u_lightDirection", lightDirection );
    
    shaderProgram.SetUniform( "u_ambient", 0.2f );
    
    shaderProgram.UnUse();
}

void GLModelShader::SetAmbient( float ambient )
{
    shaderProgram.SetUniform( "u_ambient", ambient );
}

void GLModelShader::SetLightDirection( const Eigen::Vector3d &lightDirection )
{
    shaderProgram.SetUniform( "u_lightDirection", lightDirection );
}

void GLModelShader::SetModelViewProj( const Eigen::Matrix4d &mvp )
{
    shaderProgram.SetUniform( "u_mvp", mvp );
}

GLModel::~GLModel()
{
    for ( GLuint i = 0; i < textures.size(); i++ ) {
        glDeleteTextures( 1, &textures[i] );
    }
    delete drawable;
}

struct SortByTexture
{
    bool operator()( const GLModelObject &a, const GLModelObject &b ) { return ( a.texID < b.texID ); }
};

void GLModel::SortObjectsByTexture()
{
    sort( objects.begin(), objects.end(), SortByTexture() );
}

void GLModel::Render()
{
    GLuint texID = 0;
    
    for ( vector<GLModelObject>::iterator it = objects.begin(); it != objects.end(); it++ )
    {
        if ( it->texID != texID ) {
            glBindTexture( GL_TEXTURE_2D, it->texID );
            texID = it->texID;
        }
        drawable->Draw( it->type, it->index, it->length );
    }
}

void GLModel::RenderNoTextureBind()
{
    for ( vector<GLModelObject>::iterator it = objects.begin(); it != objects.end(); it++ )
    {
        drawable->Draw( it->type, it->index, it->length );
    }
}

