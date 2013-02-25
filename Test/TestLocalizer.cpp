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
#include <FeatureMatcher/bruteforce.h>
#include <FeatureExtraction/features.h>

#include <cvd/image_io.h>
#include <cvd/draw.h>
#include <cvd/timer.h>
#include <cvd/image_interpolate.h>
#include <cvd/vision.h>
#include <cvd/convolution.h>

#include <TooN/Lapack_Cholesky.h>

using namespace vrlt;
using namespace std;
using namespace CVD;
using namespace TooN;

void loadImages( string prefix, Node *node )
{
    if ( node->camera != NULL ) {
        stringstream path;
        path << prefix << "/" << node->camera->path;
        node->camera->image = img_load( path.str() );
        node->camera->pyramid = ImagePyramid( node->camera );
    }
    
    ElementList::iterator it;
    for ( it = node->children.begin(); it != node->children.end(); it++ )
    {
        Node *child = (Node *)it->second;
        loadImages( prefix, child );
    }
}

int main( int argc, char **argv )
{
    if ( argc != 4 && argc != 5 ) {
        fprintf( stderr, "usage: %s <reconstruction> <query> <step> [<test bounds>]\n", argv[0] );
        exit(1);
    }
    
    string pathin = string(argv[1]);
    string queryin = string(argv[2]);
    int step = atoi(argv[3]);
    bool test_bounds = ( argc == 5 );
    
    SimpleTimer readtimer( "read reconstruction", 1 );
    
    readtimer.click();
    Reconstruction r;
    r.pathPrefix = pathin;
    stringstream mypath;
    mypath << pathin << "/reconstruction.xml";
    XML::read( r, mypath.str() );
    
    Node *root = (Node*)r.nodes["root"];
    XML::readDescriptors( r, root );
    readtimer.click();

    SimpleTimer readimagestimer( "read images", 1 );
    readimagestimer.click();
    loadImages( pathin, root );
    readimagestimer.click();
    
    double minY = INFINITY;
    for ( ElementList::iterator it = root->children.begin(); it != root->children.end(); it++ ) {
        Node *node = (Node *)it->second;
        Vector<3> center = -( node->globalPose().get_rotation().inverse() * node->globalPose().get_translation() );
        if ( center[1] < minY ) minY = center[1];
    }

    Reconstruction query;
    XML::read( query, queryin );
    
    Node *queryroot = new Node;
    queryroot->name = "root";

    query.nodes[ "root"] = queryroot;
    
    ElementList::iterator it;
    
    SimpleTimer buildtimer( "build localizer", 1 );
    buildtimer.click();
    NN *index = NULL;

    index = new BruteForceNN;
    
    NNLocalizer *localizer = new NNLocalizer( root, index );
    buildtimer.click();
    localizer->verbose = true;
    localizer->tracker->verbose = false;
    ElementList::iterator camerait = query.cameras.begin();
    Camera *camera = (Camera*)camerait->second;
    Calibration *calibration = camera->calibration;
    Image<byte> image = img_load( camera->path );
    int width = image.size().x;
    localizer->thresh = 0.006 * width / camera->calibration->focal;
    localizer->tracker->minnumpoints = 100;
    
    SimpleTimer feature_timer( "extract features", 1 );
    SimpleTimer localizer_timer( "localize", 1 );
    
    Calibration *mycalibration = new Calibration;
    Camera *mycamera = new Camera;
    Node *mynode = new Node;
    mycamera->calibration = calibration;
    mycamera->node = mynode;
    mynode->camera = mycamera;
    mynode->parent = NULL;
    
    int count = 0;
    int goodCount = 0;
    int attemptedCount = 0;
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
        mycamera->image = img_load( mycamera->path );
        mycamera->pyramid.resize( mycamera->image.size() );
        mycamera->pyramid.copy_from( mycamera->image );
        
        mycamera->features.clear();
        vector<Feature*> features;
        feature_timer.click();
		extractSIFT( mycamera->image, features );

        feature_timer.click();
        for ( int i = 0; i < features.size(); i++ ) 
        {
            char name[256];
            sprintf( name, "feature%d", i );
            features[i]->name = name;
            features[i]->camera = mycamera;
            mycamera->features[name] = features[i];
        }
        
        bool good = localizer->localize( mycamera );
        
        if ( good && test_bounds ) {
            SE3<> pose = mycamera->node->pose;
            Vector<3> center = - ( pose.get_rotation().inverse() * pose.get_translation() );
            if ( center[1] > 0 || center[1] < minY ) {
                cout << "\trejected because of bad pose (Y = " << center[1] << " and minY is " << minY << ")\n";
                good = false;
            }
        }
        
        if ( good ) goodCount++;
        attemptedCount++;
        
        if ( querycamera->node->parent != NULL ) querycamera->node->parent->pose = SE3<>();
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
    }
    
    XML::write( query, "localized.xml" );
    
    return 0;
}
