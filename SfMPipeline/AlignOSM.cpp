/*
 * Copyright (c) 2014. Jonathan Ventura. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: AlignOSM.cpp
 * Author: Jonathan Ventura
 * Last Modified: 27.4.2014
 */

#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include "osm_xml_reader.h"

#include <opencv2/highgui.hpp>

#include <iostream>

using namespace vrlt;

Reconstruction r;
Node *root = NULL;

double minX;
double centerX;
double maxX;
double minZ;
double centerZ;
double maxZ;

double pointangle = 0.0;
double pointscale = 1.0;

OSMData osmdata;

double mineast;
double centereast;
double maxeast;
double minnorth;
double centernorth;
double maxnorth;

double osmscale = 1.0;

const int size = 1000;

void getPointLimits()
{
    minX = INFINITY;
    maxX = -INFINITY;
    minZ = INFINITY;
    maxZ = -INFINITY;
    
    std::vector<double> Xvalues;
    std::vector<double> Zvalues;
    Xvalues.reserve( root->points.size() );
    Zvalues.reserve( root->points.size() );
    
    for ( ElementList::iterator it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point*)it->second;
        
        double X = point->position[0] / point->position[3];
        double Z = point->position[2] / point->position[3];
        
        Xvalues.push_back( X );
        Zvalues.push_back( Z );
        
//        if ( X < minX ) minX = X;
//        if ( X > maxX ) maxX = X;
//        if ( Z < minZ ) minZ = Z;
//        if ( Z > maxZ ) maxZ = Z;
    }
    
    std::sort( Xvalues.begin(), Xvalues.end() );
    std::sort( Zvalues.begin(), Zvalues.end() );
    
    minX = Xvalues[ Xvalues.size() * .1 ];
    centerX = Xvalues[ Xvalues.size() * .5 ];
    maxX = Xvalues[ Xvalues.size() * .9 ];
    minZ = Zvalues[ Zvalues.size() * .1 ];
    centerZ = Zvalues[ Zvalues.size() * .5 ];
    maxZ = Zvalues[ Zvalues.size() * .9 ];
    
    pointscale = size/(maxZ-minZ);
}

void renderPoints( cv::Mat &image )
{
    for ( ElementList::iterator it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point*)it->second;
        
        double X = point->position[0] / point->position[3];
        double Z = point->position[2] / point->position[3];
        
        Eigen::Vector3d XYZ;
        XYZ << X,0,Z;
        
        Eigen::Vector3d rotvec;
        rotvec << 0,pointangle,0;
        Sophus::SO3d rot = Sophus::SO3d::exp(rotvec);
        
        XYZ = rot*XYZ;
        
        cv::Point2i pt( size/2 + round(pointscale*(XYZ[0]-centerX)), size/2 - round(pointscale*(XYZ[2]-centerZ)) );
        cv::circle( image, pt, 2, cv::Scalar(0) );
    }
}

void getOSMLimits()
{
    minnorth = INFINITY;
    maxnorth = -INFINITY;
    mineast = INFINITY;
    maxeast = -INFINITY;
    
    for ( OSMWayList::iterator it = osmdata.ways.begin(); it != osmdata.ways.end(); it++ )
    {
        OSMWay *way = it->second;
        if ( way->building == false ) continue;
        
        for ( size_t i = 0; i < way->nodeids.size(); i++ )
        {
            OSMNode *node = osmdata.nodes[way->nodeids[i]];
        
            if ( node->north < minnorth ) minnorth = node->north;
            if ( node->north > maxnorth ) maxnorth = node->north;
            if ( node->east < mineast ) mineast = node->east;
            if ( node->east > maxeast ) maxeast = node->east;
        }
    }
    
    centereast = mineast+(maxeast-mineast)/2;
    centernorth = minnorth+(maxnorth-minnorth)/2;
    osmscale = size/(maxeast-mineast);
    
}

void renderOSM( cv::Mat &image )
{
    for ( OSMWayList::iterator it = osmdata.ways.begin(); it != osmdata.ways.end(); it++ )
    {
        OSMWay *way = it->second;
        if ( way->building == false ) continue;
        
        for ( size_t i = 1; i < way->nodeids.size(); i++ )
        {
            OSMNode *node0 = osmdata.nodes[way->nodeids[i-1]];
            OSMNode *node1 = osmdata.nodes[way->nodeids[i]];
            
            cv::Point2i pt0( size/2 + round(osmscale*(node0->east-centereast)), size/2 - round(osmscale*(node0->north-centernorth)) );
            cv::Point2i pt1( size/2 + round(osmscale*(node1->east-centereast)), size/2 - round(osmscale*(node1->north-centernorth)) );
            cv::line( image, pt0, pt1, cv::Scalar(0) );
        }
    }
}

int main( int argc, char **argv )
{
    if ( argc != 4 ) {
        fprintf( stderr, "usage: %s <reconstruction> <osm xml file> <output>\n", argv[0] );
        return 0;
    }
    
    std::string pathin = std::string(argv[1]);
    std::string osmin = std::string(argv[2]);
    std::string pathout = std::string(argv[3]);
    
    XML::read( r, pathin );
    
    if ( r.nodes.find("root") == r.nodes.end() )
    {
        std::cerr << "error: no root node found.\n";
        exit(1);
    }
    root = (Node*)r.nodes["root"];
    
    getPointLimits();
    
    osmdata.read( osmin );
    std::cout << "OSM data has " << osmdata.nodes.size() << " nodes and " << osmdata.ways.size() << " ways\n";
    
    getOSMLimits();
    
    double bigstep = (maxeast-mineast)/100.;
    double smallstep = (maxeast-mineast)/1000.;
    
    cv::Mat mapimage;
    mapimage = cv::Mat( cv::Size( size, size ), CV_8UC1, cv::Scalar(255) );
    
    cv::namedWindow( "AlignOSM" );
    
    
    bool should_render = true;
    bool should_quit = false;
    while ( !should_quit )
    {
        int key = cv::waitKey( 1 );
        
        switch ( key )
        {
            case 'q':
                should_quit = true;
                break;
                
            case 'k':
                centernorth += bigstep;
                should_render = true;
                break;

            case 'i':
                centernorth -= bigstep;
                should_render = true;
                break;

            case 'K':
                centernorth += smallstep;
                should_render = true;
                break;
                
            case 'I':
                centernorth -= smallstep;
                should_render = true;
                break;

            case 'j':
                centereast += bigstep;
                should_render = true;
                break;
                
            case 'l':
                centereast -= bigstep;
                should_render = true;
                break;
                
            case 'J':
                centereast += smallstep;
                should_render = true;
                break;
                
            case 'L':
                centereast -= smallstep;
                should_render = true;
                break;

            case 'w':
                osmscale *= 1.05;
                should_render = true;
                break;
                
            case 's':
                osmscale /= 1.05;
                should_render = true;
                break;
                
            case 't':
                pointangle += M_PI/180.;
                should_render = true;
                break;
                
            case 'r':
                pointangle -= M_PI/180.;
                should_render = true;
                break;
                
            case 'a':
                pointscale *= 2;
                should_render = true;
                break;
                
            case 'z':
                pointscale /= 2;
                should_render = true;
                break;

            default:
                break;
        }
        
        if ( should_render )
        {
            mapimage = cv::Scalar(255);
            
            renderOSM( mapimage );
        
            renderPoints( mapimage );
            
            cv::imshow( "AlignOSM", mapimage );
            
            should_render = false;
        }
    }
    
    cv::imwrite( "Output/map.png", mapimage );
    
    return 0;
}
