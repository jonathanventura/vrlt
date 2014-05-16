/*
 * Copyright (c) 2014. Jonathan Ventura. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: RefineAlignOSM.cpp
 * Author: Jonathan Ventura
 * Last Modified: 15.5.2014
 */

#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include "osm_xml_reader.h"

#include <opencv2/highgui.hpp>

#include <BundleAdjustment/bundle.h>

#include <iostream>

using namespace vrlt;

Reconstruction r;
Node *root = NULL;

OSMData osmdata;

Plane *ground_plane = NULL;
std::vector< std::pair<Eigen::Vector2d,Eigen::Vector2d> > lines;
std::vector< Plane* > planes;

void makePlanes()
{
    ground_plane = new Plane;
    ground_plane->name = "ground";
    ground_plane->origin = Eigen::Vector3d::Zero();
    ground_plane->normal = makeVector(0.,-1.,0.);
    ground_plane->X = makeVector(1.,0.,0.);
    ground_plane->Y = makeVector(0.,0.,1.);
    root->planes[ground_plane->name] = ground_plane;
    ground_plane->node = root;
    
    for ( OSMWayList::iterator it = osmdata.ways.begin(); it != osmdata.ways.end(); it++ )
    {
        OSMWay *way = it->second;
        if ( way->building == false ) continue;
        
        for ( size_t i = 1; i < way->nodeids.size(); i++ )
        {
            OSMNode *node0 = osmdata.nodes[way->nodeids[i-1]];
            OSMNode *node1 = osmdata.nodes[way->nodeids[i]];
            
            Eigen::Vector2d line_pt0;
            line_pt0 << node0->east-r.utmCenterEast,node0->north-r.utmCenterNorth;
            Eigen::Vector2d line_pt1;
            line_pt1 << node1->east-r.utmCenterEast,node1->north-r.utmCenterNorth;
            
            lines.push_back( std::make_pair( line_pt0, line_pt1 ) );
            
            Plane *plane = new Plane;
            char name[256];
            sprintf( name, "plane.%lu", planes.size() );
            plane->name = name;
            planes.push_back(plane);
            root->planes[plane->name] = plane;
            plane->node = root;
            
            Eigen::Vector2d v = line_pt1 - line_pt0;
            double d = v.norm();
            v /= d;
            
            Eigen::Vector2d n;
            n << v[1], -v[0];
            
            Eigen::Vector2d o = line_pt1;
            
            Eigen::Vector3d myn;
            myn << n[0], 0, n[1];
            Eigen::Vector3d myo;
            myo << o[0], 0, o[1];
            
            plane->normal = myn;
            plane->origin = myo;
            
            Eigen::Vector3d planeX;
            Eigen::Vector3d planeY;
            
            // figure out best XY vectors
            double dY = fabs(plane->normal.dot(makeVector(0.,1.,0.)));
            double dZ = fabs(plane->normal.dot(makeVector(0.,0.,1.)));
            if ( dY < dZ )
            {
                planeY = makeVector(0.,1.,0.);
            }
            else
            {
                planeY = makeVector(0.,0.,1.);
            }
            
            planeX = plane->normal.cross(planeY);
            planeX.normalize();
            planeY = plane->normal.cross(planeX);
            planeY.normalize();
            
            plane->X = planeX;
            plane->Y = planeY;
        }
    }
}

void addOSMPlanesToReconstruction()
{
    size_t num_on_building = 0;
    size_t num_on_ground = 0;
    size_t num_not_on_plane = 0;

    for ( ElementList::iterator it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point*)it->second;
        
        Eigen::Vector3d pt = project( point->position );
        
        Eigen::Vector4d XYZ;
        XYZ << pt[0], pt[1], pt[2], 1;
        
        Eigen::Vector2d transformed_pt;
        transformed_pt << XYZ[0],XYZ[2];
        
        // find nearest line
        double min_dist = INFINITY;
        size_t best = 0;
        for ( size_t i = 0; i < lines.size(); i++ )
        {
            Eigen::Vector2d v = lines[i].second - lines[i].first;
            double d = v.norm();
            v /= d;
            
            Eigen::Vector2d n;
            n << v[1], -v[0];
            
            Eigen::Vector2d o = lines[i].first;
            
            double dist = fabs(n.dot(transformed_pt-o));
            
            //            Eigen::Vector3d myn;
            //            myn << n[0], 0, n[1];
            //            Eigen::Vector3d myo;
            //            myo << o[0], 0, o[1];
            //            PointPlaneError err(pt,myn,myo);
            //            double mydist;
            //            err(params,&mydist);
            //            std::cout << "correct: " << dist << "   ceres: " << mydist << "\n";
            
            if ( dist < min_dist )
            {
                // check projection onto line
                double proj = v.dot(transformed_pt-o);
                if ( proj >= 0 && proj <= d )
                {
                    min_dist = dist;
                    best = i;
                }
            }
        }
        
        if ( min_dist <= 4. ) {
            point->plane = planes[best];
            num_on_building++;
        } else if ( fabs(XYZ[1])<0.5 ) {
            point->plane = ground_plane;
            num_on_ground++;
        } else {
            num_not_on_plane++;
        }
    }
    
    std::cout << "num points on a building: " << num_on_building << "\n";
    std::cout << "num points on the ground: " << num_on_ground << "\n";
    std::cout << "num points not on a plane: " << num_not_on_plane << "\n";
}

void removePlanes()
{
    for ( ElementList::iterator it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point *)it->second;
        point->plane = NULL;
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
    
    osmdata.read( osmin );
    std::cout << "OSM data has " << osmdata.nodes.size() << " nodes and " << osmdata.ways.size() << " ways\n";
    
    makePlanes();
    
    for ( int i = 0; i < 10; i++ )
    {
        addOSMPlanesToReconstruction();
        
        // make all nodes free, not fixed
        for ( ElementList::iterator it = root->children.begin(); it != root->children.end(); it++ )
        {
            Node *node = (Node*)it->second;
            node->fixed = false;
        }
        
        Bundle bundle( root, true );
        bundle.run();

        removePlanes();
        
        // fix the first node again
        Node *firstnode = (Node*)root->children.begin()->second;
        firstnode->fixed = true;
        
        Bundle pointbundle( root, root->children, ElementList(), true );
        pointbundle.run();
    }
    
    XML::write( r, pathout );
    
    return 0;
}
