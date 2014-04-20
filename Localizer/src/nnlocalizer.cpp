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
#include <FeatureMatcher/bruteforce.h>

#include <opencv2/imgproc.hpp>

#include <dispatch/dispatch.h>

#include <iostream>

static const int min_level = 0;

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
    
    static void drawMatches( Node *querynode, std::vector<Match*> &matches, std::vector<bool> &inliers )
    {
        /*
        std::vector<Camera*> cameras;
        std::vector< cv::Mat* > images;
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
            Image<byte> *image = NULL;
            for ( int k = 0; k < cameras.size(); k++ ) {
                if ( feature->camera == cameras[k] ) {
                    found = true;
                    image = images[k];
                    break;
                }
            }
            if ( image == NULL ) {
                cameras.push_back( feature->camera );
                Image<byte> im = feature->camera->image;
                int height = ( queryim.size().y > im.size().y ) ? queryim.size().y : im.size().y;
                Image<byte> *composite = new Image<byte>( ImageRef( im.size().x+queryim.size().x, height ) );
                composite->sub_image( ImageRef(0,0), queryim.size() ).copy_from( queryim );
                composite->sub_image( ImageRef(queryim.size().x,0), im.size() ).copy_from( im );
                images.push_back( composite );
                image = images.back();
            }
            
            ImageRef impt1( queryfeature->location[0], queryfeature->location[1] );
            ImageRef impt2( queryim.size().x+feature->location[0], feature->location[1] );
            
            drawCross( *image, impt1, queryfeature->scale, (byte)0 );
            drawCross( *image, impt2, feature->scale, (byte)0 );
            
            drawLine( *image, impt1, impt2, (byte)0 );
            
        }
        
        for ( int j = 0; j < cameras.size(); j++ ) {
            char name[256];
            sprintf( name, "Output/%s_%s_matches.jpg", querynode->name.c_str(), cameras[j]->name.c_str() );
            img_save( *(images[j]), name, ImageType::JPEG );
            delete images[j];
        }
        */
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
        
        findMatches( (*fm), features, matches );

        std::vector<bool> inliers;

        std::map<std::string,float> scores;
        for ( int i = 0; i < matches.size(); i++ )
        {
            std::string trackname = matches[i]->feature1->track->name;
            if ( scores.count(trackname) == 0 ) scores[trackname] = matches[i]->score;
            else if ( matches[i]->score < scores[trackname] ) scores[trackname] = matches[i]->score;
        }
        
        PointPairList all_point_pairs;
        PointPairList point_pairs;
        for ( int i = 0; i < matches.size(); i++ )
        {
            Match *match = matches[i];
            
            Feature *feature = match->feature1;
            Feature *queryfeature = match->feature2;
            
            if ( match->score != scores[feature->track->name] ) continue;
            
            PointPair point_pair;
            point_pair.first = project( feature->track->point->position );
            point_pair.second = queryfeature->unproject();
            
            int querylevel = (int) log2f( queryfeature->scale / 1.6 );
            if ( querylevel >= min_level ) point_pairs.push_back( point_pair );
            all_point_pairs.push_back( point_pair );
        }
        
        std::cout << point_pairs.size() << " matches for pose estimation; " << all_point_pairs.size() << " matches for inliers\n";
        
        ThreePointPose estimator;
        
        PROSAC prosac;
        prosac.num_trials = 5000;
        prosac.min_num_inliers = 100;
        prosac.inlier_threshold = thresh;
        
        int ninliers = prosac.compute( point_pairs.begin(), point_pairs.end(), all_point_pairs.begin(), all_point_pairs.end(), estimator, inliers );
        
        ElementList::iterator it;
        std::cout << matches.size() << " matches; " << ninliers << " PROSAC inliers\n";
        
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
        querycamera->node->pose = estimator.pose;
        bool good = false;
        for ( int i = 0; i < 10; i++ )
        {
            Sophus::SE3d last_pose = querycamera->node->pose;
            tracker->verbose = false;
            good = tracker->track( querycamera );
            std::cout << "tracker tracked " << tracker->ntracked << " / " << tracker->nattempted << "\n";
            if ( !good ) break;
            if ( good ) {
                robustlsq.run( querycamera, 10 );
                Sophus::SE3d update = querycamera->node->pose * last_pose.inverse();
                if ( update.log().norm() < 1e-6 ) break;
            }
        }

        return good;
    }
}
