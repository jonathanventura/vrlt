/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: TestTracker.cpp
 * Author: Jonathan Ventura
 * Last Modified: 25.2.2013
 */

#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <PatchTracker/tracker.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <PatchTracker/robustlsq.h>
#include <BundleAdjustment/updatepose.h>

#include <GeographicLib/UTMUPS.hpp>

#include <iostream>

using namespace vrlt;

void drawPoints( Node *root, Camera *camera, bool good )
{
    /*
    //Image< Rgb<byte> > image_out = img_load( camera->path );
    cv::Mat image_out( camera->image.size(), CV_8UC3, cv::Scalar(0,0,0) );
    cv::cvtColor( camera->image, image_out, cv::COLOR_RGB2GRAY );
    
    std::vector<cv::Point2i> circle[NLEVELS];
    for ( int i = 0; i < NLEVELS; i++ ) circle[i] = getCircle( 4*powf(2,i) );
    
    ElementList::iterator it;
    for ( it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point *)it->second;
        //if ( !point->tracked ) continue;
        
        // check in front
        Vector<3> PX = camera->node->globalPose() * project( point->position );
        if ( PX[2] < 0 ) continue;
        
        // check in image
        Vector<2> location = camera->calibration->project( camera->calibration->distort( project( PX ) ) );
        ImageRef pos( (int)location[0], (int)location[1] );
        if ( !camera->image.in_image( pos ) ) continue;
        
        Rgb<byte> pointcolor( 255, 0, 0 );
        if ( point->tracked ) {
            if ( good ) pointcolor = Rgb<byte>(0,255,0);
            else pointcolor = Rgb<byte>(255,255,0);
        }
        
        if ( point->tracked && good ) drawLine( image_out, location, point->location, Rgb<byte>(255,255,255) );
        drawBox( image_out, pos - ImageRef(2,2), pos + ImageRef(2,2), pointcolor );
        drawShape( image_out, pos, circle[point->bestLevel], pointcolor );
        //drawCross( image_out, pos, 5, ( good ) ? Rgb<byte>( 255, 255, 0 ) : Rgb<byte>( 255, 0, 0 ) );
    }
    
    char name[256];
    sprintf( name, "Output/%s.jpg", camera->name.c_str() );
    img_save( image_out, name, ImageType::JPEG );
    */
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
    if ( argc != 3 && argc != 4 ) {
        fprintf( stderr, "usage: %s <reconstruction> <localized query> [<gps file>]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string queryin = std::string(argv[2]);
    bool have_gps = ( argc == 4 );
    std::vector<Eigen::Vector2d> gps_locations;
    std::vector<Eigen::Vector2d> tracker_locations;
    if ( have_gps )
    {
        std::string gpsin = std::string(argv[3]);
        loadGPS( gpsin, gps_locations );
        std::cout << "read " << gps_locations.size() << " GPS locations\n";
    }

    // load reconstruction
    Reconstruction r;
    r.pathPrefix = pathin;
    std::stringstream mypath;
    mypath << pathin << "/reconstruction.xml";
    XML::read( r, mypath.str() );
    Node *root = (Node*)r.nodes["root"];
    
    // set up tracker
    int maxnumpoints = 1024;
    double minratio = 0.2;
    Tracker tracker( root, maxnumpoints, "ncc", 0.7 );
    tracker.verbose = false;
    tracker.firstlevel = 3;
    tracker.lastlevel = 0;
    tracker.niter = 10;
    tracker.minnumpoints = 20;
    tracker.minratio = minratio;
    tracker.do_pose_update = false;
    
    // load images
    ElementList::iterator it;
    for ( it = r.cameras.begin(); it != r.cameras.end(); it++ )
    {
        Camera *camera = (Camera *)it->second;
        std::stringstream path;
        path << pathin << "/" << camera->path;
        camera->image = cv::imread( path.str(), cv::IMREAD_GRAYSCALE );
        camera->pyramid = ImagePyramid( camera );
    }
    
    // load query
    Reconstruction query;
    XML::read( query, queryin, false );
    Node *queryroot = (Node*)query.nodes["root"];
    if ( queryroot == NULL )
    {
        queryroot = new Node;
        queryroot->name = "root";
        query.nodes["root"] = queryroot;
    }
    
    const int nmaxlost = 3;
    int nlost = nmaxlost;
    
    // use one camera object for the tracker
    // so we don't allocate a new image for each frame
    Camera *trackercamera = new Camera;
    
    Eigen::Matrix3d mymat;
    mymat <<
    0, -1, 0,
    -1, 0, 0,
    0, 0,-1;
    Sophus::SO3d gyroconversion(mymat);
    
    int count = 0;
    Sophus::SE3d localizer_pose;
    Sophus::SO3d localizer_attitude;
    double prev_timestamp;
    Sophus::SO3d prev_attitude;
    
    Sophus::SE3d last_pose;

    std::vector<Camera*> trackedCameras;
    
    RobustLeastSq robustlsq( root );
    
    int framesSinceLocalized = 0;

    for ( it = query.cameras.begin(); it != query.cameras.end(); it++,count++ )
    {
        Camera *querycamera = (Camera*)it->second;
        Node *querynode = (Node *)querycamera->node;
        
        // get gyro input
        Sophus::SO3d queryattitude = gyroconversion * querycamera->attitude * gyroconversion;
        double querytimestamp = querycamera->timestamp;
        
        prev_attitude = queryattitude;
        prev_timestamp = querytimestamp;
        
        Eigen::Vector3d gyroxyz = querycamera->rotationRate;
        Eigen::Vector3d gyrox, gyroy, gyroz;
        gyrox << gyroxyz[0], 0, 0;
        gyroy << 0, gyroxyz[1], 0;
        gyroz << 0, 0, gyroxyz[2];
        
        Sophus::SO3d gyro( Sophus::SO3d::exp( gyroz ) * Sophus::SO3d::exp( gyroy ) * Sophus::SO3d::exp( gyrox ) );
        gyro = gyroconversion * gyro * gyroconversion;
        
        // create query node if necessary
        if ( querynode == NULL )
        {
            querynode = new Node;
            char name[256];
            sprintf( name, "%s.node", querycamera->name.c_str() );
            querynode->name = name;
            query.nodes[ querynode->name ] = querynode;
            querycamera->node = querynode;
            querynode->camera = querycamera;
        }
        
        trackercamera->name = querycamera->name;
        trackercamera->path = querycamera->path;
        trackercamera->node = querynode;
        trackercamera->calibration = querycamera->calibration;
        trackercamera->velocity = querycamera->velocity;

        if ( querynode->root() == queryroot && nlost >= nmaxlost ) {
            last_pose = querynode->pose;
            framesSinceLocalized = 0;
        }
        
        // get predicted pose
        querynode->pose = last_pose;
        
        // load the image
        trackercamera->image = cv::imread( querycamera->path, cv::IMREAD_GRAYSCALE );
        trackercamera->pyramid.resize( trackercamera->image.size() );
        trackercamera->pyramid.copy_from( trackercamera->image );
        
        // run tracker
        bool good = tracker.track( trackercamera );
        std::cout << "tracked: " << tracker.ntracked << " / " << tracker.nattempted << " (" << tracker.ntracked / (float) tracker.nattempted << ")\n";
		int ntracked = 0;
		float newratio = (float)tracker.nnew / (float)tracker.ntracked;
		if ( good ) {
            good = robustlsq.run( trackercamera );
            good = updatePose( root, trackercamera );
            if ( good ) {
                ElementList::iterator it;
                for ( it = root->points.begin(); it != root->points.end(); it++ )
                {
                    Point *point = (Point *)it->second;
                    if ( point->tracked ) ntracked++;
                }
                float ratio = (float) ntracked / tracker.nattempted;
                std::cout << "after optimization, tracked: " << ntracked << " / " << tracker.nattempted << " (" << ntracked / (float) tracker.nattempted << ")\n";
                good = ( ratio > tracker.minratio );
            }
        }

        if ( good ) {
            nlost = 0;
            
            trackedCameras.push_back( querycamera );
            last_pose = querycamera->node->pose;

			bool should_add = true;
            
			// add a new camera if we have few new points,
			// or if we are far from other new cameras
            if ( newratio >= .5 ) {
                should_add = false;
                
                Sophus::SE3d trackerpose = querycamera->node->pose;
                Eigen::Vector3d trackercenter = -( trackerpose.so3().inverse() * trackerpose.translation() );
                
                double min_dist = INFINITY;
                
                // check distance to other new cameras
                for ( int i = 0; i < tracker.cameras.size(); i++ )
                {
                    Camera *camera = tracker.cameras[i];
                    if ( camera->isnew == false ) continue;
                    
                    Sophus::SE3d pose = camera->node->pose;
                    Eigen::Vector3d center = -( pose.so3().inverse() * pose.translation() );
                    
                    double dist = ( center - trackercenter ).norm();
                    if ( dist < min_dist ) min_dist = dist;
                }
                
                if ( min_dist > 2. ) should_add = true;
            }
            
            if ( framesSinceLocalized < 30 ) should_add = false;
	
            // un-comment to disable keyframe sampling
            //should_add = false;
            if ( should_add ) {
				Camera *newcamera = NULL;
                newcamera = addCameraToReconstruction( r, querycamera->calibration,
														trackercamera->image, querycamera->node->pose );
                tracker.cameras.push_back( newcamera );
            }
            
            framesSinceLocalized++;
        } else {
            nlost++;
            
            if ( querynode->root() == queryroot )
            {
                // remove from root
                queryroot->children.erase( querynode->name );
                querynode->parent = NULL;
                query.nodes[ querynode->name ] = querynode;
            }
        }
        
        if ( good && have_gps )
        {
            Sophus::SE3d pose = querycamera->node->globalPose();
            Eigen::Vector3d center = -(pose.so3().inverse()*pose.translation());
            Eigen::Vector2d location = makeVector(center[0],center[2]);
            tracker_locations.push_back( location );
        }
        
        //drawPoints( root, trackercamera, good );
    }
    
    if ( have_gps )
    {
        FILE *f = NULL;
        
        f = fopen("gps_locations.obj","w");
        for ( size_t i = 0; i < gps_locations.size(); i++ )
        {
            fprintf( f, "v %.17lf 0 %.17lf\n", gps_locations[i][0]-r.utmCenterEast, gps_locations[i][1]-r.utmCenterNorth );
        }
        fclose(f);
        
        f = fopen("tracker_locations.obj","w");
        for ( size_t i = 0; i < tracker_locations.size(); i++ )
        {
            fprintf( f, "v %.17lf 0 %.17lf\n", tracker_locations[i][0], tracker_locations[i][1] );
        }
        fclose(f);
    }
    
    // move tracked cameras to root
    for ( int i = 0; i < trackedCameras.size(); i++ )
    {
        Camera *camera = trackedCameras[i];
        Node *node = camera->node;
        
        if ( node->parent != NULL ) continue;
        
        queryroot->children[ node->name ] = node;
        node->parent = queryroot;
        query.nodes.erase( node->name );
    }
    
    XML::write( query, "tracked.xml" );

    return 0;
}
