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

#include <ceres/ceres.h>

#include <sophus/sim3.hpp>

#include <iostream>

using namespace vrlt;

Reconstruction r;
Node *root = NULL;

OSMData osmdata;

struct Transformation
{
    double centerX;
    double centerZ;
    double angle;    // radians
    double scale;
    Transformation() : centerX(0),centerZ(0),angle(0),scale(1) { }
    Eigen::Matrix4d matrix()
    {
        Eigen::Vector3d r;
        r << 0, angle, 0;
        Eigen::Matrix4d mat( Eigen::Matrix4d::Identity() );
        mat.block<3,3>(0,0) = Sophus::SO3d::exp(r).matrix();
        Eigen::Vector3d c;
        c << centerX,0,centerZ;
        mat.block<3,1>(0,3) = mat.block<3,3>(0,0)*(-c);
        Eigen::Vector4d s;
        s << scale,scale,scale,1;
        return Eigen::DiagonalMatrix<double, 4>(s)*mat;
    }
    Sophus::Sim3d sim3()
    {
        Sophus::Sim3d T;
        
        Eigen::Vector3d r;
        r << 0, angle, 0;
        
        Sophus::SO3d R( Sophus::SO3d::exp(r) );
        
        T.rxso3().setScaledRotationMatrix( Eigen::DiagonalMatrix<double, 3>(scale,scale,scale)*R.matrix());
        
        Eigen::Vector3d c;
        c << centerX,0,centerZ;
        
        T.translation() = scale*(R*(-c));
        
        return T;
    }
};

Transformation pointTransformation;
Transformation osmTransformation;

double metersToPixels = 1.0;

const int size = 1000;

std::map<Point*,double> point_scores;

struct PointLineError
{
    PointLineError( const Eigen::Vector2d &pt, const Eigen::Vector2d &line_pt0, const Eigen::Vector2d &line_pt1 )
    {
        x = pt[0];
        y = pt[1];
        Eigen::Vector2d v = line_pt0-line_pt1;
        v.normalize();
        nx = v[1];
        ny = -v[0];
        lx = line_pt0[0];
        ly = line_pt0[1];
    }
    
    template <typename T>
    bool operator()(const T* const params,
                    T* residuals) const
    {
        // params: centerX, centerZ, angle, scale
        
        // remove center
        T ptx = T(x)-params[0];
        T pty = T(y)-params[1];
        
        // apply rotation
        T c = cos(params[2]);
        T s = sin(params[2]);
        T Rptx = c*ptx+s*pty;
        T Rpty = -s*ptx+c*pty;
        
        // apply scale
        T sRptx = params[3]*Rptx;
        T sRpty = params[3]*Rpty;
        
        // compute point-line distance
        residuals[0] = nx*(lx-sRptx)+ny*(ly-sRpty);
        
        return true;
    }

    double x, y;    // point
    double nx, ny; // line normal
    double lx, ly; // line origin
};

void optimizeAlignment()
{
    ceres::Problem problem;
    
    std::vector< std::pair<Eigen::Vector2d,Eigen::Vector2d> > lines;
    
    for ( OSMWayList::iterator it = osmdata.ways.begin(); it != osmdata.ways.end(); it++ )
    {
        OSMWay *way = it->second;
        if ( way->building == false ) continue;
        
        for ( size_t i = 1; i < way->nodeids.size(); i++ )
        {
            OSMNode *node0 = osmdata.nodes[way->nodeids[i-1]];
            OSMNode *node1 = osmdata.nodes[way->nodeids[i]];
            
            Eigen::Vector4d XYZ0;
            XYZ0 << node0->east,0,node0->north,1;
            XYZ0 = osmTransformation.matrix() * XYZ0;
            
            Eigen::Vector4d XYZ1;
            XYZ1 << node1->east,0,node1->north,1;
            XYZ1 = osmTransformation.matrix() * XYZ1;
            
            Eigen::Vector2d line_pt0;
            line_pt0 << XYZ0[0], XYZ0[2];
            Eigen::Vector2d line_pt1;
            line_pt1 << XYZ1[0], XYZ1[2];
            
            lines.push_back( std::make_pair( line_pt0, line_pt1 ) );
        }
    }
    
    double params[4];
    params[0] = pointTransformation.centerX;
    params[1] = pointTransformation.centerZ;
    params[2] = pointTransformation.angle;
    params[3] = pointTransformation.scale;

    ceres::HuberLoss *lossFunction = new ceres::HuberLoss(0.5);
    
    for ( ElementList::iterator it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point*)it->second;
        
        Eigen::Vector2d pt;
        pt << point->position[0] / point->position[3], point->position[2] / point->position[3];
        
        Eigen::Vector4d XYZ;
        XYZ << pt[0],0,pt[1],1;
        
        XYZ = pointTransformation.matrix() * XYZ;
        
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
            double dist = fabs(n.dot(transformed_pt-lines[i].first));
            
            if ( dist < min_dist )
            {
                // check projection onto line
                double proj = v.dot(transformed_pt-lines[i].first);
                if ( proj >= 0 && proj <= d )
                {
                    min_dist = dist;
                    best = i;
                }
            }
        }
        
        point_scores[point] = min_dist;
        
        if ( min_dist > 4. ) continue;
        
        PointLineError *err = new PointLineError(pt,lines[best].first,lines[best].second);
        problem.AddResidualBlock( new ceres::AutoDiffCostFunction<PointLineError, 1, 4>(err), lossFunction, params );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    if ( summary.termination_type != ceres::FAILURE )
    {
        pointTransformation.centerX = params[0];
        pointTransformation.centerZ = params[1];
        pointTransformation.angle = params[2];
        pointTransformation.scale = params[3];
    }
}

void getPointLimits()
{
    double minX = INFINITY;
    double maxX = -INFINITY;
    double minZ = INFINITY;
    double maxZ = -INFINITY;
    
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
        
        point_scores[point] = 1.;
    }
    
    std::sort( Xvalues.begin(), Xvalues.end() );
    std::sort( Zvalues.begin(), Zvalues.end() );
    
    minX = Xvalues[ Xvalues.size() * .1 ];
    pointTransformation.centerX = Xvalues[ Xvalues.size() * .5 ];
    maxX = Xvalues[ Xvalues.size() * .9 ];
    minZ = Zvalues[ Zvalues.size() * .1 ];
    pointTransformation.centerZ = Zvalues[ Zvalues.size() * .5 ];
    maxZ = Zvalues[ Zvalues.size() * .9 ];
    
    pointTransformation.scale = 100./(maxZ-minZ);
}

void renderPoints( cv::Mat &image )
{
    for ( ElementList::iterator it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point*)it->second;
        
        double X = point->position[0] / point->position[3];
        double Z = point->position[2] / point->position[3];
        
        Eigen::Vector4d XYZ;
        XYZ << X,0,Z,1;
        
        XYZ = pointTransformation.matrix() * XYZ;
        
        cv::Point2i pt( size/2 + round(metersToPixels*XYZ[0]), size/2 - round(metersToPixels*XYZ[2]) );
        double score = point_scores[point];
        
        if ( score < 1. ) cv::circle( image, pt, 0, cv::Scalar(0) );
        else cv::circle( image, pt, 0, cv::Scalar(128) );
    }
}

void getOSMLimits()
{
    double minnorth = INFINITY;
    double maxnorth = -INFINITY;
    double mineast = INFINITY;
    double maxeast = -INFINITY;
    
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
    
    osmTransformation.centerX = mineast+(maxeast-mineast)/2;
    osmTransformation.centerZ = minnorth+(maxnorth-minnorth)/2;
    metersToPixels = size/(maxeast-mineast);
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
            
            Eigen::Vector4d XYZ0;
            XYZ0 << node0->east,0,node0->north,1;
            XYZ0 = osmTransformation.matrix() * XYZ0;
            
            Eigen::Vector4d XYZ1;
            XYZ1 << node1->east,0,node1->north,1;
            XYZ1 = osmTransformation.matrix() * XYZ1;

            cv::Point2i pt0( size/2 + round(metersToPixels*XYZ0[0]), size/2 - round(metersToPixels*XYZ0[2]) );
            cv::Point2i pt1( size/2 + round(metersToPixels*XYZ1[0]), size/2 - round(metersToPixels*XYZ1[2]) );
            cv::line( image, pt0, pt1, cv::Scalar(0) );
        }
    }
}

struct ObjFace
{
    size_t v0,v1,v2;
    ObjFace( size_t _v0, size_t _v1, size_t _v2 ) : v0(_v0), v1(_v1), v2(_v2) { }
};

void writeOSMObj( const char *path )
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<ObjFace> faces;
    
    std::map<size_t,size_t> nodemap;
    
    for ( OSMNodeList::iterator it = osmdata.nodes.begin(); it != osmdata.nodes.end(); it++ )
    {
        OSMNode *node = it->second;
        nodemap[node->ID] = vertices.size();
        
        Eigen::Vector4d XYZ;
        XYZ << node->east,0,node->north,1;
        XYZ = osmTransformation.matrix() * XYZ;

        vertices.push_back( XYZ.head(3)/XYZ[3] );
    }
    
    for ( OSMWayList::iterator it = osmdata.ways.begin(); it != osmdata.ways.end(); it++ )
    {
        OSMWay *way = it->second;
        if ( way->building == false ) continue;
        
        for ( size_t i = 2; i < way->nodeids.size(); i++ )
        {
            faces.push_back( ObjFace( nodemap[way->nodeids[i-2]], nodemap[way->nodeids[i-1]], nodemap[way->nodeids[i]] ) );
        }
    }
    
    FILE *f = fopen(path,"w");
    
    for ( size_t i = 0; i < vertices.size(); i++ )
    {
        fprintf( f, "v %lf %lf %lf\n", vertices[i][0], vertices[i][1], vertices[i][2] );
    }

    for ( size_t i = 0; i < faces.size(); i++ )
    {
        fprintf( f, "f %lu %lu %lu\n", faces[i].v0+1, faces[i].v1+1, faces[i].v2+1 );
    }

    fclose(f);
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
    
    double bigstep = 1.;
    double smallstep = 0.1;
    
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
                osmTransformation.centerZ += bigstep;
                should_render = true;
                break;

            case 'i':
                osmTransformation.centerZ -= bigstep;
                should_render = true;
                break;

            case 'K':
                osmTransformation.centerZ += smallstep;
                should_render = true;
                break;
                
            case 'I':
                osmTransformation.centerZ -= smallstep;
                should_render = true;
                break;

            case 'j':
                osmTransformation.centerX += bigstep;
                should_render = true;
                break;
                
            case 'l':
                osmTransformation.centerX -= bigstep;
                should_render = true;
                break;
                
            case 'J':
                osmTransformation.centerX += smallstep;
                should_render = true;
                break;
                
            case 'L':
                osmTransformation.centerX -= smallstep;
                should_render = true;
                break;

            case 'w':
                pointTransformation.scale *= 1.05;
                should_render = true;
                break;
                
            case 's':
                pointTransformation.scale /= 1.05;
                should_render = true;
                break;
                
            case 't':
                pointTransformation.angle += M_PI/180.;
                should_render = true;
                break;
                
            case 'r':
                pointTransformation.angle -= M_PI/180.;
                should_render = true;
                break;
                
            case 'a':
                metersToPixels *= 2;
                should_render = true;
                break;
                
            case 'z':
                metersToPixels /= 2;
                should_render = true;
                break;
                
            case 'b':
                optimizeAlignment();
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
    
    r.utmZone = osmdata.utm_zone;
    r.utmNorth = osmdata.utm_north;
    r.utmCenterEast = osmTransformation.centerX;
    r.utmCenterNorth = osmTransformation.centerZ;
    
    Transformation myOsmTransformation = osmTransformation;
    myOsmTransformation.centerX = 0;
    myOsmTransformation.centerZ = 0;
    
    Sophus::Sim3d composedTransform = myOsmTransformation.sim3().inverse()*pointTransformation.sim3();
    
    transformPoints( root, composedTransform );
    
    XML::write( r, pathout );
    
    writeOSMObj( "osm.obj" );
    
    cv::imwrite( "Output/map.png", mapimage );
    
    return 0;
}
