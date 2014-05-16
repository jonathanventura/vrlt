/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
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
#include <iostream>

#include <Eigen/Core>

#include <opencv2/imgproc.hpp>

#if defined( __IPHONE__ )
#include <GLUtils/LoadImage.h>
#else
#include <opencv2/highgui.hpp>
#endif

static inline Eigen::Vector2d scaleVector( const Eigen::Vector2d &scale, const Eigen::Vector2d &in )
{
    Eigen::Vector2d output;
    output << scale[0] * in[0], scale[1] * in[1];
    return output;
}

static inline Eigen::Vector2d rotateVector( const Eigen::Vector2d &in )
{
    Eigen::Vector2d output;
    output << in[1], in[0];
    return output;
}

struct Texture
{
    std::string name;
    cv::Mat image;
    bool hasCoords;
    
    // for texture packing
    Eigen::Vector2d scale;
    Eigen::Vector2d offset;
    bool rotated;
    
    Eigen::Vector2d transformTexCoords( const Eigen::Vector2d &tc )
    {
        if ( !rotated )
            return scaleVector( scale, tc ) + offset;
        else
            return scaleVector( scale, rotateVector( tc ) ) + offset;
    }
};

cv::Mat PackTextures( std::vector<Texture> &textures )
{
    TEXTURE_PACKER::TexturePacker *tp = TEXTURE_PACKER::createTexturePacker();
    tp->setTextureCount( textures.size() );
    for ( int i = 0; i < textures.size(); i++ ) {
        tp->addTexture( textures[i].image.size().width, textures[i].image.size().height );
    }
    int width, height;
    tp->packTextures( width, height, true, false );
    
    cv::Mat packed_texture( cv::Size( width, height ), CV_8UC3, cv::Scalar(255,255,255) );
    printf( "packed texture size: %d %d\n", width, height );
    for ( int i = 0; i < textures.size(); i++ ) {
        int tx, ty, tw, th;
        bool rotated = tp->getTextureLocation( i, tx, ty, tw, th );
        
        if ( !rotated ) {
            textures[i].scale << (float)textures[i].image.size().width / (float)width, (float)textures[i].image.size().height / (float)height;
        } else {
            textures[i].scale << (float)textures[i].image.size().height / (float)width, (float)textures[i].image.size().width / (float)height;
        }
        textures[i].offset << (float)tx / (float)width, (float)ty / (float)height;
        textures[i].rotated = rotated;

        if ( rotated ) {
            int w = textures[i].image.size().width;
            int h = textures[i].image.size().height;
            cv::Mat rotated_im( cv::Size( h, w ), CV_8UC3 );
            cv::flip(textures[i].image,rotated_im,-1);
//            for ( int y = 0; y < h; y++ ) {
//                for ( int x = 0; x < w; x++ ) {
//                    rotated_im.at<cv::Vec3b>(x,y) = textures[i].image.at<cv::Vec3b>(y,x);
//                }
//            }
            rotated_im.copyTo( packed_texture( cv::Rect(cv::Point2i(tx,ty),rotated_im.size()) ) );
        } else {
            textures[i].image.copyTo( packed_texture( cv::Rect(cv::Point2i(tx,ty),textures[i].image.size()) ) );
        }
    }

    TEXTURE_PACKER::releaseTexturePacker( tp );
    return packed_texture;
}

void LoadMaterialFile( const char *prefix, const char *filename, std::vector<Texture> &textures )
{
    char path[256];
    sprintf( path, "%s/%s", prefix, filename );
    
    FILE *f = fopen( path, "r" );
    
    if ( f == NULL )
    {
        std::cerr << "error: could not open material file " << path << "\n";
        exit(1);
    }
    
    char buffer[256];

    std::string mtlname;
    cv::Scalar Kd;
    bool found_map = false;

    while ( !feof( f ) ) {
        if ( fgets( buffer, 256, f ) == NULL ) break;
        if ( buffer[0] == '#' ) continue;

        std::istringstream line( buffer );
        std::string word;
        line >> word;
        
        if ( word == "newmtl" ) {
            if ( found_map == false && mtlname.length() > 0 ) {
                // create texture with Kd color
                Texture texture;
                texture.name = mtlname;
                texture.hasCoords = false;
                texture.image = cv::Mat( cv::Size(1,1), CV_8UC3, Kd );
                textures.push_back( texture );
            }
            found_map = false;
            line >> mtlname;
        } else if ( word == "Kd" ) {
            float r,g,b;
            line >> r >> g >> b;
            Kd.val[0] = 255 * r;
            Kd.val[1] = 255 * g;
            Kd.val[2] = 255 * b;
        } else if ( word == "map_Kd" ) {
            found_map = true;
            
            std::string mtlpath;
            line >> mtlpath;
            
            Texture texture;
            
            texture.name = mtlname;
            texture.hasCoords = true;
            
            sprintf( path, "%s/%s", prefix, mtlpath.c_str() );
#if defined( __IPHONE__ )
            loadJPEG( path, texture.image );
#else
            texture.image = cv::imread( path, cv::IMREAD_COLOR );
#endif
            if ( texture.image.empty() )
            {
                std::cerr << "could not load texture " << path << "\n";
                exit(1);
            }
//            cout << path << " has dimensions " << texture.image.size().x << " " << texture.image.size().y << "\n";
//            cout << (int) texture.image.data()[0].red << "\n";
            cv::flip( texture.image, texture.image, 0 );
            
            textures.push_back( texture );
        }
    }
    
    if ( found_map == false && mtlname.length() > 0 ) {
        // create texture with Kd color
        Texture texture;
        texture.name = mtlname;
        texture.hasCoords = false;
        texture.image = cv::Mat( cv::Size(1,1), CV_8UC3, Kd );
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
    
    std::vector<Texture> textures;
    cv::Mat packed_texture;
    GLuint packedTexID;
    std::map<std::string,GLuint> materials;
    
    // load materials
    while ( !feof( f ) ) {
        if ( fgets( buffer, 256, f ) == NULL ) break;
        if ( buffer[0] == '#' ) continue;
        
        std::istringstream line( buffer );
        std::string word;
        line >> word;
        
        if ( word == "mtllib" ) {
            std::string material_path;
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
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, packed_texture.size().width, packed_texture.size().height, 0, GL_BGR, GL_UNSIGNED_BYTE, packed_texture.data );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    checkGL( "make packed texture: packed texture is probably too big." );
    rewind( f );
    
    
    
    
    std::vector< Eigen::Vector3d > vertices;
    std::vector< Eigen::Vector2d > texCoords;
    std::vector< Eigen::Vector3d > normals;
    
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
        
        std::istringstream line( buffer );
        std::string word;
        line >> word;
        
        if ( word == "v" ) {
            Eigen::Vector3d data;
            line >> data[0] >> data[1] >> data[2];
            vertices.push_back( data );
        } else if ( word == "vt" ) {
            Eigen::Vector2d data;
            line >> data[0] >> data[1];
            texCoords.push_back( data );
        } else if ( word == "vn" ) {
            Eigen::Vector3d data;
            line >> data[0] >> data[1] >> data[2];
            normals.push_back( data );
        } else if ( word == "usemtl" ) {
            std::string data;
            line >> data;
            current_texture = NULL;
            for ( int i = 0; i < textures.size(); i++ ) if ( textures[i].name == data ) current_texture = &textures[i];
        } else if ( word == "f" ) {
            if ( current_texture == NULL ) continue;
            
            std::string data;
            int v,t,n;
            Eigen::Vector3d vertex;
            Eigen::Vector2d texCoord;
            Eigen::Vector3d normal;
            Eigen::Vector4d scaleOffset;
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
                    model->drawable->AddElem( vertex.cast<float>(), texCoord.cast<float>(), scaleOffset.cast<float>(), normal.cast<float>() );
                }
            } else {
                for ( int i = 0; i < 3; i++ ) {
                    line >> data;
                    sscanf( data.c_str(), "%d//%d", &v, &n );
                    vertex = vertices[v-1];
                    if ( i == 0 ) texCoord << 0, 0;
                    else if ( i == 1 ) texCoord << 1, 0;
                    else if ( i == 2 ) texCoord << 0, 1;
                    normal = normals[n-1];
                    model->drawable->AddElem( vertex.cast<float>(), texCoord.cast<float>(), scaleOffset.cast<float>(), normal.cast<float>() );
                }
            }
            
            model->objects.back().length += 3;
        }
    }

    std::cout << "model has " << model->objects.size() << " objects\n";
    std::cout << "model has " << model->objects.back().length << " faces\n";
    std::cout << "model texture is " << model->objects[0].texID << "\n";
    
    if ( model->objects.back().length == 0 )
    {
        // assume point cloud
        model->objects.back().type = GL_POINTS;
        
        if ( current_texture != NULL ) {
            for ( int i = 0; i < vertices.size(); i++ ) {
                Eigen::Vector3d vertex;
                Eigen::Vector2d texCoord;
                Eigen::Vector4d scaleOffset;
                scaleOffset[0] = current_texture->scale[0];
                scaleOffset[1] = current_texture->scale[1];
                scaleOffset[2] = current_texture->offset[0];
                scaleOffset[3] = current_texture->offset[1];
                vertex = vertices[i];
                if ( !current_texture->rotated ) texCoord = texCoords[i];
                else texCoord = rotateVector( texCoords[i] );
                model->drawable->AddElem( vertex.cast<float>(), texCoord.cast<float>(), scaleOffset.cast<float>() );
            }
            model->objects.back().length = vertices.size();
        }
    }
    
    model->drawable->Commit();
    
    fclose( f );
    
    return model;
}
