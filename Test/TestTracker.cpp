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

#include <cvd/image_io.h>
#include <cvd/draw.h>
#include <cvd/timer.h>

#include <PatchTracker/robustlsq.h>

using namespace vrlt;
using namespace std;
using namespace CVD;
using namespace TooN;

void drawPoints( Node *root, Camera *camera, bool good )
{
    //Image< Rgb<byte> > image_out = img_load( camera->path );
    Image< Rgb<byte> > image_out( camera->image.size(), Rgb<byte>(0,0,0) );
    convert_image( camera->image, image_out );
    
    vector<ImageRef> circle[NLEVELS];
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
}

int main( int argc, char **argv )
{
    if ( argc != 3 && argc != 4 ) {
        fprintf( stderr, "usage: %s <reconstruction> <localized query>\n", argv[0] );
        exit(1);
    }
    
    string pathin = string(argv[1]);
    string queryin = string(argv[2]);
    
    // load reconstruction
    Reconstruction r;
    r.pathPrefix = pathin;
    stringstream mypath;
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
        stringstream path;
        path << pathin << "/" << camera->path;
        camera->image = img_load( path.str() );
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
    
    SimpleTimer tracktimer("track",100);
    
    TooN::Matrix<3> mymat;
    mymat[0] = TooN::makeVector( 0,-1, 0 );
    mymat[1] = TooN::makeVector(-1, 0, 0 );
    mymat[2] = TooN::makeVector( 0, 0,-1 );
    SO3<> gyroconversion = TooN::SO3<>( mymat );
    
    int count = 0;
    SE3<> localizer_pose;
    SO3<> localizer_attitude;
    double prev_timestamp;
    SO3<> prev_attitude;
    
    SE3<> last_pose;

    vector<Camera*> trackedCameras;
    
    RobustLeastSq robustlsq( root );
    
    int framesSinceLocalized = 0;

    for ( it = query.cameras.begin(); it != query.cameras.end(); it++,count++ )
    {
        Camera *querycamera = (Camera*)it->second;
        Node *querynode = (Node *)querycamera->node;
        
        // get gyro input
        SO3<> queryattitude = gyroconversion * querycamera->attitude * gyroconversion;
        double querytimestamp = querycamera->timestamp;
        
        prev_attitude = queryattitude;
        prev_timestamp = querytimestamp;
        
        Vector<3> gyroxyz = querycamera->rotationRate;
        TooN::SO3<> gyro( TooN::SO3<>( TooN::makeVector( 0, 0, gyroxyz[2] ) ) * TooN::SO3<>( TooN::makeVector( 0, gyroxyz[1], 0 ) ) * TooN::SO3<>( TooN::makeVector( gyroxyz[0], 0, 0 ) ) );
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
        trackercamera->image = img_load( querycamera->path );
        trackercamera->pyramid.resize( trackercamera->image.size() );
        trackercamera->pyramid.copy_from( trackercamera->image );
        
        // run tracker
        tracktimer.click();
        bool good = tracker.track( trackercamera );
        cout << "tracked: " << tracker.ntracked << " / " << tracker.nattempted << " (" << tracker.ntracked / (float) tracker.nattempted << ")\n";
		int ntracked = 0;
		float newratio = (float)tracker.nnew / (float)tracker.ntracked;
		if ( good ) {
            good = robustlsq.run( trackercamera, 10 );
            if ( good ) {
                ElementList::iterator it;
                for ( it = root->points.begin(); it != root->points.end(); it++ )
                {
                    Point *point = (Point *)it->second;
                    if ( point->tracked ) ntracked++;
                }
                float ratio = (float) ntracked / tracker.nattempted;
                good = ( ratio > tracker.minratio );
            }
        }
        tracktimer.click();
        
        if ( good ) {
            nlost = 0;
            
            trackedCameras.push_back( querycamera );
            last_pose = querycamera->node->pose;

			bool should_add = true;
            
			// add a new camera if we have few new points,
			// or if we are far from other new cameras
            if ( newratio >= .5 ) {
                should_add = false;
                
                SE3<> trackerpose = querycamera->node->pose;
                Vector<3> trackercenter = -( trackerpose.get_rotation().inverse() * trackerpose.get_translation() );
                
                double min_dist = INFINITY;
                
                // check distance to other new cameras
                for ( int i = 0; i < tracker.cameras.size(); i++ )
                {
                    Camera *camera = tracker.cameras[i];
                    if ( camera->isnew == false ) continue;
                    
                    SE3<> pose = camera->node->pose;
                    Vector<3> center = -( pose.get_rotation().inverse() * pose.get_translation() );
                    
                    double dist = norm( center - trackercenter );
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
        
        //drawPoints( root, trackercamera, good );
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
