/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: TestLocalizer.cpp
 * Author: Jonathan Ventura
 * Last Modified: 25.2.2013
 */

#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <Localizer/localizer.h>
#include <Localizer/nnlocalizer.h>
#include <FeatureMatcher/approxnn.h>
#include <FeatureExtraction/features.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Eigen>

#include <GeographicLib/UTMUPS.hpp>

#include <iostream>

using namespace vrlt;

void loadImages( std::string prefix, Node *node )
{
    if ( node->camera != NULL ) {
        std::stringstream path;
        path << prefix << "/" << node->camera->path;
        node->camera->image = cv::imread( path.str(), cv::IMREAD_GRAYSCALE );
        node->camera->pyramid = ImagePyramid( node->camera );
    }
    
    ElementList::iterator it;
    for ( it = node->children.begin(); it != node->children.end(); it++ )
    {
        Node *child = (Node *)it->second;
        loadImages( prefix, child );
    }
}

void loadGPS( const std::string &path, std::vector<Eigen::Vector2d> &gps_locations )
{
    FILE *f = fopen( path.c_str(), "r" );
    if ( f == NULL )
    {
        std::cerr << "error: could not read GPS file at " << path << "\n";
        return;
    }
    
    while ( !feof( f ) )
    {
        double lat, lon;
        int nread = fscanf( f, "%lf,%lf\n",&lat,&lon);
        if ( nread != 2 ) break;
        
        int utm_zone;
        bool utm_north;
        double east;
        double north;
        double gamma;
        double k;
        GeographicLib::UTMUPS::Forward(lat, lon,
                                       utm_zone, utm_north,
                                       east, north,
                                       gamma, k );
        
        gps_locations.push_back( makeVector( east, north ) );
    }
    
    fclose(f);
}

int main( int argc, char **argv )
{
    if ( argc != 4 && argc != 5 ) {
        fprintf( stderr, "usage: %s <reconstruction> <query> <step> [<gps file>]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string queryin = std::string(argv[2]);
    int step = atoi(argv[3]);
    bool have_gps = ( argc == 5 );
    std::vector<Eigen::Vector2d> gps_locations;
    std::vector<Eigen::Vector2d> localizer_locations;
    if ( have_gps )
    {
        std::string gpsin = std::string(argv[4]);
        loadGPS( gpsin, gps_locations );
        std::cout << "read " << gps_locations.size() << " GPS locations\n";
    }
    
    Reconstruction r;
    r.pathPrefix = pathin;
    std::stringstream mypath;
    mypath << pathin << "/reconstruction.xml";
    XML::read( r, mypath.str() );
    
    Node *root = (Node*)r.nodes["root"];
    XML::readDescriptors( r, root );

    loadImages( pathin, root );
    
    double minY = INFINITY;
    for ( ElementList::iterator it = root->children.begin(); it != root->children.end(); it++ ) {
        Node *node = (Node *)it->second;
        Eigen::Vector3d center = -( node->globalPose().so3().inverse() * node->globalPose().translation() );
        if ( center[1] < minY ) minY = center[1];
    }

    Reconstruction query;
    XML::read( query, queryin );
    
    Node *queryroot = new Node;
    queryroot->name = "root";

    query.nodes[ "root"] = queryroot;
    
    ElementList::iterator it;
    
    NN *index = NULL;

    index = new ApproxNN;
    
    NNLocalizer *localizer = new NNLocalizer( root, index );
    localizer->verbose = true;
    localizer->tracker->verbose = false;
    ElementList::iterator camerait = query.cameras.begin();
    Camera *camera = (Camera*)camerait->second;
    Calibration *calibration = camera->calibration;
    cv::Mat image = cv::imread( camera->path, cv::IMREAD_GRAYSCALE );
    int width = image.size().width;
    localizer->thresh = 0.006 * width / camera->calibration->focal;
    localizer->tracker->minnumpoints = 100;
    
    Camera *mycamera = new Camera;
    Node *mynode = new Node;
    mycamera->calibration = calibration;
    mycamera->node = mynode;
    mynode->camera = mycamera;
    mynode->parent = NULL;
    
    std::vector<Eigen::Vector2d> gps_errors;
    
    int count = 0;
    int goodCount = 0;
    int attemptedCount = 0;
    int gpsCounter = 0;
    for ( it = query.cameras.begin(); it != query.cameras.end(); it++,count++ )
    {
        if ( count % step != 0 ) continue;
        
        Camera *querycamera = (Camera *)it->second;
        
        Node *querynode = (Node *)querycamera->node;
        if ( querynode == NULL )
        {
            querynode = new Node;
            char name[256];
            sprintf( name, "%s.node", querycamera->name.c_str() );
            querynode->name = name;
            query.nodes[ querynode->name ] = querynode;
            querynode->camera = querycamera;
            querycamera->node = querynode;
        }

        
        mycamera->name = querycamera->name;
        mynode->pose = querycamera->node->globalPose();
        mycamera->path = querycamera->path;
        mycamera->image = cv::imread( mycamera->path, cv::IMREAD_GRAYSCALE );
        mycamera->pyramid.resize( mycamera->image.size() );
        mycamera->pyramid.copy_from( mycamera->image );
        
        mycamera->features.clear();
        std::vector<Feature*> features;
		extractSIFT( mycamera->image, features );

        for ( int i = 0; i < features.size(); i++ )
        {
            char name[256];
            sprintf( name, "feature%d", i );
            features[i]->name = name;
            features[i]->camera = mycamera;
            mycamera->features[name] = features[i];
        }
        
        bool good = localizer->localize( mycamera );
                
        if ( good ) goodCount++;
        attemptedCount++;
        
        if ( querycamera->node->parent != NULL ) querycamera->node->parent->pose = Sophus::SE3d();
        querycamera->node->pose = mynode->pose;

        for ( int i = 0; i < features.size(); i++ )
        {
            delete features[i];
        }

        if ( good ) {
            query.nodes.erase( querycamera->node->name );
            querycamera->node->parent = queryroot;
            queryroot->children[ querycamera->node->name ] = querycamera->node;
        }
        
        if ( have_gps && good )
        {
            Sophus::SE3d querypose = querycamera->node->globalPose();
            Eigen::Vector3d query_center = -(querypose.so3().inverse() * querypose.translation());
            double error_east = query_center[0] - (gps_locations[gpsCounter][0] - r.utmCenterEast);
            double error_north = query_center[2] - (gps_locations[gpsCounter][1] - r.utmCenterNorth);
            gps_errors.push_back( makeVector( error_east, error_north ) );
            std::cout << "gps error: " << error_east << " " << error_north << "\n";
            localizer_locations.push_back( makeVector( query_center[0], query_center[2] ) );
        }
    }
    
    if ( have_gps )
    {
        FILE *f = fopen("gps_errors.txt","w");
        for ( size_t i = 0; i < gps_errors.size(); i++ )
        {
            fprintf( f, "%.17lf %.17lf\n", gps_errors[i][0], gps_errors[i][1] );
        }
        fclose(f);
        
        f = fopen("gps_locations.obj","w");
        for ( size_t i = 0; i < gps_locations.size(); i++ )
        {
            fprintf( f, "v %.17lf 0 %.17lf\n", gps_locations[i][0]-r.utmCenterEast, gps_locations[i][1]-r.utmCenterNorth );
        }
        fclose(f);

        f = fopen("localizer_locations.obj","w");
        for ( size_t i = 0; i < localizer_locations.size(); i++ )
        {
            fprintf( f, "v %.17lf 0 %.17lf\n", localizer_locations[i][0], localizer_locations[i][1] );
        }
        fclose(f);
    }
    
    XML::write( query, "localized.xml" );
    
    return 0;
}
