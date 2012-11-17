/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: LoadOBJ.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/CheckGL.h>
#include <GLUtils/GLModel.h>
#include <GLUtils/GLGenericDrawable.h>
#include <GLUtils/TexturePacker.h>

#include <map>
#include <vector>
#include <string>
#include <sstream>
using namespace std;

#include <TooN/TooN.h>
using namespace TooN;

#include <cvd/vision.h>
using namespace CVD;

#if defined( __IPHONE__ )
#include <GLUtils/LoadImage.h>
#else
#include <cvd/image_io.h>
#endif

Vector<2> scaleVector( const Vector<2> &scale, const Vector<2> &in )
{
    return makeVector( scale[0] * in[0], scale[1] * in[1] );
}

Vector<2> rotateVector( const Vector<2> &in )
{
    return makeVector( in[1], in[0] );
}

struct Texture
{
    string name;
    Image< Rgb<byte> > image;
    bool hasCoords;
    
    // for texture packing
    Vector<2> scale;
    Vector<2> offset;
    bool rotated;
    
    Vector<2> transformTexCoords( const Vector<2> &tc )
    {
        if ( !rotated )
            return scaleVector( scale, tc ) + offset;
        else
            return scaleVector( scale, rotateVector( tc ) ) + offset;
    }
};

Image< Rgb<byte> > PackTextures( vector<Texture> &textures )
{
    TEXTURE_PACKER::TexturePacker *tp = TEXTURE_PACKER::createTexturePacker();
    tp->setTextureCount( textures.size() );
    for ( int i = 0; i < textures.size(); i++ ) {
        tp->addTexture( textures[i].image.size().x, textures[i].image.size().y );
    }
    int width, height;
    tp->packTextures( width, height, true, false );
    
    Image< Rgb<byte> > packed_texture( ImageRef( width, height ), Rgb<byte>(255,255,255) );
    printf( "packed texture size: %d %d\n", width, height );
    for ( int i = 0; i < textures.size(); i++ ) {
        int tx, ty, tw, th;
        bool rotated = tp->getTextureLocation( i, tx, ty, tw, th );
        
        if ( !rotated ) {
            textures[i].scale = makeVector( (float)textures[i].image.size().x / (float)width, (float)textures[i].image.size().y / (float)height );
        } else {
            textures[i].scale = makeVector( (float)textures[i].image.size().y / (float)width, (float)textures[i].image.size().x / (float)height );
        }
        textures[i].offset = makeVector( (float)tx / (float)width, (float)ty / (float)height );
        textures[i].rotated = rotated;

        if ( rotated ) {
            int w = textures[i].image.size().x;
            int h = textures[i].image.size().y;
            Image< Rgb<byte> > rotated_im( ImageRef( h, w ) );
            for ( int y = 0; y < h; y++ ) {
                for ( int x = 0; x < w; x++ ) {
                    rotated_im[x][y] = textures[i].image[y][x];
                }
            }
            packed_texture.sub_image( ImageRef(tx,ty), rotated_im.size() ).copy_from( rotated_im );
        } else {
            packed_texture.sub_image( ImageRef(tx,ty), textures[i].image.size() ).copy_from( textures[i].image );
        }
    }

    TEXTURE_PACKER::releaseTexturePacker( tp );
    return packed_texture;
}

void LoadMaterialFile( const char *prefix, const char *filename, vector<Texture> &textures )
{
    char path[256];
    sprintf( path, "%s/%s", prefix, filename );
    
    FILE *f = fopen( path, "r" );
    
    char buffer[256];

    string mtlname;
    Rgb<byte> Kd;
    bool found_map = false;

    while ( !feof( f ) ) {
        if ( fgets( buffer, 256, f ) == NULL ) break;
        if ( buffer[0] == '#' ) continue;

        istringstream line( buffer );
        string word;
        line >> word;
        
        if ( word == "newmtl" ) {
            if ( found_map == false && mtlname.length() > 0 ) {
                // create texture with Kd color
                Texture texture;
                texture.name = mtlname;
                texture.hasCoords = false;
                texture.image.resize( ImageRef(1,1) );
                texture.image.data()[0] = Kd;
                textures.push_back( texture );
            }
            found_map = false;
            line >> mtlname;
        } else if ( word == "Kd" ) {
            float r,g,b;
            line >> r >> g >> b;
            Kd.red = 255 * r;
            Kd.green = 255 * g;
            Kd.blue = 255 * b;
        } else if ( word == "map_Kd" ) {
            found_map = true;
            
            string mtlpath;
            line >> mtlpath;
            
            Texture texture;
            
            texture.name = mtlname;
            texture.hasCoords = true;
            
            sprintf( path, "%s/%s", prefix, mtlpath.c_str() );
#if defined( __IPHONE__ )
            loadJPEG( path, texture.image );
#else
            texture.image = img_load( path );
#endif
//            cout << path << " has dimensions " << texture.image.size().x << " " << texture.image.size().y << "\n";
//            cout << (int) texture.image.data()[0].red << "\n";
            flipVertical( texture.image );
            
            textures.push_back( texture );
        }
    }
    
    if ( found_map == false && mtlname.length() > 0 ) {
        // create texture with Kd color
        Texture texture;
        texture.name = mtlname;
        texture.hasCoords = false;
        texture.image.resize( ImageRef(1,1) );
        texture.image.data()[0] = Kd;
        textures.push_back( texture );
    }

    fclose( f );
}

GLModel * LoadOBJ( const char *prefix, const char *filename )
{
    GLModel *model = new GLModel;
    
    char path[256];
    sprintf( path, "%s/%s", prefix, filename );
    
    FILE *f = fopen( path, "r" );
    
    char buffer[256];
    
    vector<Texture> textures;
    Image< Rgb<byte> > packed_texture;
    GLuint packedTexID;
    map<string,GLuint> materials;
    
    // load materials
    while ( !feof( f ) ) {
        if ( fgets( buffer, 256, f ) == NULL ) break;
        if ( buffer[0] == '#' ) continue;
        
        istringstream line( buffer );
        string word;
        line >> word;
        
        if ( word == "mtllib" ) {
            string material_path;
            line >> material_path;
            LoadMaterialFile( prefix, material_path.c_str(), textures );
        }
    }

    packed_texture = PackTextures( textures );
    //img_save( packed_texture, "Output/packed.png", ImageType::PNG );
        
    // create packed texture
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei( GL_PACK_ALIGNMENT, 1 );
    //glPixelStorei(GL_UNPACK_ROW_LENGTH, 0 );
    
    checkGL( "about to make packed texture" );
    glGenTextures( 1, &packedTexID );
    glBindTexture( GL_TEXTURE_2D, packedTexID );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, packed_texture.size().x, packed_texture.size().y, 0, GL_RGB, GL_UNSIGNED_BYTE, packed_texture.data() );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    checkGL( "make packed texture: packed texture is probably too big." );
    rewind( f );
    
    
    
    
    vector< Vector<3> > vertices;
    vector< Vector<2> > texCoords;
    vector< Vector<3> > normals;
    
    model->drawable = new GLGenericDrawable();
    model->drawable->Create();
    
    model->drawable->AddAttrib( 0, 3 );
    model->drawable->AddAttrib( 1, 2 );
    model->drawable->AddAttrib( 2, 4 );
    model->drawable->AddAttrib( 3, 3 );
    
    model->objects.push_back( GLModelObject() );
    
    model->objects.back().texID = packedTexID;
    model->objects.back().index = 0;
    model->objects.back().length = 0;
    model->objects.back().type = GL_TRIANGLES;
    
    Texture *current_texture = NULL;
    
    // load vertex data
    while ( !feof( f ) ) {
        if ( fgets( buffer, 256, f ) == NULL ) break;
        if ( buffer[0] == '#' ) continue;
        
        istringstream line( buffer );
        string word;
        line >> word;
        
        if ( word == "v" ) {
            Vector<3> data;
            line >> data[0] >> data[1] >> data[2];
            vertices.push_back( data );
        } else if ( word == "vt" ) {
            Vector<2> data;
            line >> data[0] >> data[1];
            texCoords.push_back( data );
        } else if ( word == "vn" ) {
            Vector<3> data;
            line >> data[0] >> data[1] >> data[2];
            normals.push_back( data );
        } else if ( word == "usemtl" ) {
            string data;
            line >> data;
            current_texture = NULL;
            for ( int i = 0; i < textures.size(); i++ ) if ( textures[i].name == data ) current_texture = &textures[i];
        } else if ( word == "f" ) {
            if ( current_texture == NULL ) continue;
            
            string data;
            int v,t,n;
            Vector<3> vertex;
            Vector<2> texCoord;
            Vector<3> normal;
            Vector<4> scaleOffset;
            scaleOffset[0] = current_texture->scale[0];
            scaleOffset[1] = current_texture->scale[1];
            scaleOffset[2] = current_texture->offset[0];
            scaleOffset[3] = current_texture->offset[1];
            
            if ( current_texture->hasCoords ) {
                for ( int i = 0; i < 3; i++ ) {
                    line >> data;
                    sscanf( data.c_str(), "%d/%d/%d", &v, &t, &n );
                    vertex = vertices[v-1];
                    if ( !current_texture->rotated ) texCoord = texCoords[t-1];
                    else texCoord = rotateVector( texCoords[t-1] );
                    normal = normals[n-1];
                    model->drawable->AddElem( vertex, texCoord, scaleOffset, normal );
                }
            } else {
                for ( int i = 0; i < 3; i++ ) {
                    line >> data;
                    sscanf( data.c_str(), "%d//%d", &v, &n );
                    vertex = vertices[v-1];
                    if ( i == 0 ) texCoord = makeVector(0,0);
                    else if ( i == 1 ) texCoord = makeVector(1,0);
                    else if ( i == 2 ) texCoord = makeVector(0,1);
                    normal = normals[n-1];
                    model->drawable->AddElem( vertex, texCoord, scaleOffset, normal );
                }
            }
            
            model->objects.back().length += 3;
        }
    }

    cout << "model has " << model->objects.size() << " objects\n";
    cout << "model has " << model->objects.back().length << " faces\n";
    cout << "model texture is " << model->objects[0].texID << "\n";
    
    if ( model->objects.back().length == 0 )
    {
        // assume point cloud
        model->objects.back().type = GL_POINTS;
        
        if ( current_texture != NULL ) {
            for ( int i = 0; i < vertices.size(); i++ ) {
                Vector<3> vertex;
                Vector<2> texCoord;
                Vector<4> scaleOffset;
                scaleOffset[0] = current_texture->scale[0];
                scaleOffset[1] = current_texture->scale[1];
                scaleOffset[2] = current_texture->offset[0];
                scaleOffset[3] = current_texture->offset[1];
                vertex = vertices[i];
                if ( !current_texture->rotated ) texCoord = texCoords[i];
                else texCoord = rotateVector( texCoords[i] );
                model->drawable->AddElem( vertex, texCoord, scaleOffset );
            }
            model->objects.back().length = vertices.size();
        }
    }
    
    model->drawable->Commit();
    
    fclose( f );
    
    return model;
}
