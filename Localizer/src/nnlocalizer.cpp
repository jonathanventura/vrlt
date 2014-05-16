/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: nnlocalizer.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <Localizer/nnlocalizer.h>
#include <Estimator/estimator.h>
#include <PatchTracker/tracker.h>
#include <PatchTracker/robustlsq.h>
#include <BundleAdjustment/updatepose.h>

#include <opencv2/imgproc.hpp>

#include <iostream>

namespace vrlt
{
    NNLocalizer::NNLocalizer( Node *_root, NN *index ) : Localizer( _root )
    {
        fm = new FeatureMatcher( index );
        fm->init( _root, true, true );
    }
    
    NNLocalizer::~NNLocalizer()
    {
        delete fm;
    }
    
    static void drawMatches( Node *querynode, std::vector<Match*> &matches, const std::vector<bool> &inliers )
    {
        std::vector<Camera*> cameras;
        std::vector<cv::Mat> images;
        cv::Mat queryim = querynode->camera->image;
        
        for ( int j = 0; j < matches.size(); j++ )
        {
            if ( !inliers.empty() )
            {
                if ( !inliers[j] ) continue;
            }
            
            Match *match = matches[j];
            Feature *feature = match->feature1;
            Feature *queryfeature = match->feature2;
            
            bool found = false;
            cv::Mat image;
            for ( int k = 0; k < cameras.size(); k++ ) {
                if ( feature->camera == cameras[k] ) {
                    found = true;
                    image = images[k];
                    break;
                }
            }
            if ( image.empty() ) {
                cameras.push_back( feature->camera );
                cv::Mat im = feature->camera->image;
                int height = ( queryim.size().height > im.size().height ) ? queryim.size().height : im.size().height;
                cv::Mat composite( cv::Size( im.size().width+queryim.size().width, height ), CV_8UC1 );
                queryim.copyTo( composite( cv::Rect(cv::Point2i(0,0), queryim.size() ) ) );
                im.copyTo( composite( cv::Rect(cv::Point2i(queryim.size().width,0), im.size() ) ) );
                images.push_back( composite );
                image = images.back();
            }
            
            cv::Point2i impt1( queryfeature->location[0], queryfeature->location[1] );
            cv::Point2i impt2( queryim.size().width+feature->location[0], feature->location[1] );
            
            cv::line( image, impt1, impt2, cv::Scalar(0) );
        }
        
        for ( int j = 0; j < cameras.size(); j++ ) {
            char name[256];
            sprintf( name, "Output/%s_%s_matches.jpg", querynode->name.c_str(), cameras[j]->name.c_str() );
            cv::imwrite( name, images[j] );
        }
    }
    
    struct SortByFeatureName
    {
        bool operator()( Match *a, Match *b ) { return ( a->feature1->name > b->feature1->name ); }
    };
    
    bool NNLocalizer::localize( Camera *querycamera )
    {
        features.clear();
        addFeatures( querycamera->node, false, features );
        if ( features.empty() ) return false;
        
        std::vector<Match*> matches;
        
        //findMatches( (*fm), features, matches );
        findUniqueMatches( (*fm), features, 0.8, matches );

        std::vector<bool> inliers;
        
        PointPairList point_pairs;
        for ( int i = 0; i < matches.size(); i++ )
        {
            Match *match = matches[i];
            
            Feature *feature = match->feature1;
            Feature *queryfeature = match->feature2;
            
            PointPair point_pair;
            point_pair.first = project( feature->track->point->position );
            point_pair.second = queryfeature->unproject();
            
            point_pairs.push_back( point_pair );
        }
        
        std::cout << point_pairs.size() << " matches for pose estimation\n";
        
        
        Sophus::SE3d best_pose;
        int ninliers;
        
//        ThreePointPose estimator;
//        PROSAC prosac;
//        prosac.num_trials = 5000;
//        prosac.min_num_inliers = 100;
//        prosac.inlier_threshold = thresh;
//        ninliers = prosac.compute( point_pairs.begin(), point_pairs.end(), estimator, inliers );
//        best_pose = estimator.pose;
        
        std::vector<Estimator*> estimators( 5000 );
        for ( size_t i = 0; i < estimators.size(); i++ ) estimators[i] = new ThreePointPose;
        PreemptiveRANSAC preemptive_ransac( 10 );
        preemptive_ransac.inlier_threshold = thresh;
        ThreePointPose *best_estimator = NULL;
        ninliers = preemptive_ransac.compute( point_pairs.begin(), point_pairs.end(), estimators, (Estimator**)&best_estimator, inliers );
        best_pose = best_estimator->pose;
        for ( size_t i = 0; i < estimators.size(); i++ ) delete estimators[i];
        estimators.clear();
        
        ElementList::iterator it;
        std::cout << matches.size() << " matches; " << ninliers << " inliers\n";
        
//        drawMatches( querycamera->node, matches, std::vector<bool>() );//, inliers );
//        exit(1);
        for ( int i = 0; i < inliers.size(); i++ )
        {
            if ( !inliers[i] ) continue;
            
            Match *match = matches[i];
            Feature *feature = match->feature1;
            Feature *queryfeature = match->feature2;
            
            if ( queryfeature->track != NULL ) continue;
            
            Point *point = feature->track->point;
            queryfeature->track = new Track;
            queryfeature->track->point = point;
        }
        
        for ( int i = 0; i < matches.size(); i++ ) {
            delete matches[i];
        }
        
        RobustLeastSq robustlsq( root );
        querycamera->node->pose = best_pose;
        bool good = false;
        for ( int i = 0; i < 10; i++ )
        {
            Sophus::SE3d last_pose = querycamera->node->pose;
            tracker->verbose = false;
            good = tracker->track( querycamera );
            std::cout << "tracker tracked " << tracker->ntracked << " / " << tracker->nattempted << "\n";
            if ( !good ) break;
            if ( good ) {
                robustlsq.run( querycamera );
//                updatePose( root, querycamera );
                Sophus::SE3d update = querycamera->node->pose * last_pose.inverse();
                if ( update.log().norm() < 1e-6 ) break;
            }
        }

        return good;
    }
}
