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

int main( int argc, char **argv )
{
    if ( argc != 4 && argc != 5 ) {
        fprintf( stderr, "usage: %s <reconstruction> <query> <step> [<test bounds>]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string queryin = std::string(argv[2]);
    int step = atoi(argv[3]);
    bool test_bounds = ( argc == 5 );
    
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
        
        if ( good && test_bounds ) {
            Sophus::SE3d pose = mycamera->node->pose;
            Eigen::Vector3d center = - ( pose.so3().inverse() * pose.translation() );
            if ( center[1] > 0 || center[1] < minY ) {
                std::cout << "\trejected because of bad pose (Y = " << center[1] << " and minY is " << minY << ")\n";
                good = false;
            }
        }
        
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
    }
    
    XML::write( query, "localized.xml" );
    
    return 0;
}
