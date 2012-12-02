/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
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

#include <cvd/image_io.h>
#include <cvd/draw.h>

#include <cvd/timer.h>

#include <dispatch/dispatch.h>

//#define FEATURE_VERIFY
#define TRACKER_VERIFY

static const int min_level = 0;

namespace vrlt
{
    using namespace std;
    using namespace TooN;
    using namespace CVD;
    
    static SimpleTimer nntimer( "nearest neighbor", 1 );
    static SimpleTimer prosactimer( "prosac", 1 );
    static SimpleTimer verifytimer( "verify", 1 );
    static SimpleTimer tracktimer( "track", 1 );
    
    NNLocalizer::NNLocalizer( Node *_root, NN *index ) : Localizer( _root )
    {
        fm = new FeatureMatcher( index );
        fm->init( _root, true, true );
    }
    
    NNLocalizer::~NNLocalizer()
    {
        delete fm;
    }
    
    static void drawMatches( Node *querynode, vector<Match*> &matches, vector<bool> &inliers )
    {
        vector<Camera*> cameras;
        vector< Image<byte>* > images;
        Image<byte> queryim = querynode->camera->image;
        
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
        
        int k = 1;
        vector<Match*> matches;
        
        nntimer.click();
        if ( k == 1 ) {
            findMatches( (*fm), features, matches );
//            findConsistentMatches( (*nn), features, matches );
//            findUniqueMatches( (*nn), features, 0.8, matches );
        } else {
            findMatches( (*fm), features, k, matches );
        }

        nntimer.click();
        
        vector<bool> inliers;

        map<string,float> scores;
        for ( int i = 0; i < matches.size(); i++ )
        {
            string trackname = matches[i]->feature1->track->name;
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
        
        cout << point_pairs.size() << " matches for pose estimation; " << all_point_pairs.size() << " matches for inliers\n";
        
        ThreePointPose estimator;
        
        PROSAC prosac;
        prosac.num_trials = 5000;
        prosac.min_num_inliers = 100;
        prosac.inlier_threshold = thresh;
        
        prosactimer.click();
        int ninliers = prosac.compute( point_pairs.begin(), point_pairs.end(), all_point_pairs.begin(), all_point_pairs.end(), estimator, inliers );
        prosactimer.click();
        
#ifdef FEATURE_VERIFY
        verifytimer.click();
        float max_dist_sq = 0;
        for ( int i = 0; i < inliers.size(); i++ )
        {
            if ( !inliers[i] ) continue;
            
            Match *match = matches[i];
            Feature *feature = match->feature1;
            Feature *queryfeature = match->feature2;

            float dist_sq = 0;
            for ( int j = 0; j < 128; j++ ) {
                float diff = (float)feature->descriptor[j] - (float)queryfeature->descriptor[j];
                dist_sq += diff*diff;
            }
            if ( dist_sq > max_dist_sq ) max_dist_sq = dist_sq;
        }

        // SIFT-based verification
        int num_possible = 0;
        int num_feature_inliers = 0;
        ElementList::iterator it;
        for ( it = root->points.begin(); it != root->points.end(); it++ )
        {
            Point *point = (Point*)it->second;
            
            // project point into frame.
            Vector<3> PX = estimator.pose * project( point->position );
            if ( PX[2] < 0 ) continue;
            Vector<2> pos = querycamera->calibration->project( project( PX ) );
            
            //if ( pos[0] < 0 || pos[0] >= querycamera->calibration->center[0] * 2 || pos[1] < 0 || pos[1] >= querycamera->calibration->center[1] * 2 ) continue;
            
            // check all features within threshold
            ElementList::iterator queryit;
            for ( queryit = querycamera->features.begin(); queryit != querycamera->features.end(); queryit++ )
            {
                Feature *queryfeature = (Feature*)queryit->second;
                if ( queryfeature->track != NULL ) continue;
                
                double featdist = norm( queryfeature->location - pos );
                if ( featdist / querycamera->calibration->focal > thresh ) continue;

                // compare with all features observing point
                // if any feature has descriptor distance below max_dist, count as inlier.
                bool is_inlier = false;
                ElementList::iterator featit;
                for ( featit = point->track->features.begin(); featit != point->track->features.end(); featit++ )
                {
                    Feature *feature = (Feature*)featit->second;
                    
                    float descriptor_dist_sq = 0;
                    for ( int i = 0; i < 128; i++ ) {
                        float diff = (float)feature->descriptor[i] - (float)queryfeature->descriptor[i];
                        descriptor_dist_sq += diff*diff;
                    }
                    if ( descriptor_dist_sq < max_dist_sq ) {
                        is_inlier = true;
                        break;
                    }
                }
                if ( is_inlier ) {
                    num_feature_inliers++;
                    queryfeature->track = new Track;
                    queryfeature->track->point = point;
                    break;
                }
            }
        }
        cout << matches.size() << " matches; " << ninliers << " PROSAC inliers; " << num_feature_inliers << " re-found inliers\n";
        verifytimer.click();
#else
        ElementList::iterator it;
        int num_feature_inliers = ninliers;
        cout << matches.size() << " matches; " << ninliers << " PROSAC inliers\n";
        
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
#endif
        
#ifdef SEARCH_MULTIPLE
        for ( int n = 0; n < N; n++ ) {
            for ( int i = 0; i < mymatches[n].size(); i++ ) {
                delete mymatches[n][i];
            }
            mymatches[n].clear();
        }
#else
        for ( int i = 0; i < matches.size(); i++ ) {
            delete matches[i];
        }
#endif
        
#ifndef TRACKER_VERIFY
        bool good = ( num_feature_inliers >= 10 );
        
        if ( good ) {
            querycamera->node->pose = estimator.pose;
            refinePoseLM( querycamera, 10 );
        }
        
        for ( it = querycamera->features.begin(); it != querycamera->features.end(); it++ ) {
            Feature *feature = (Feature*)it->second;
            if ( feature->track != NULL ) {
                delete feature->track;
                feature->track = NULL;
            }
        }
#else
        RobustLeastSq robustlsq( root );
        tracktimer.click();
        querycamera->node->pose = estimator.pose;
        bool good = false;
        for ( int i = 0; i < 10; i++ )
        {
            SE3<> last_pose = querycamera->node->pose;
            tracker->verbose = false;
            good = tracker->track( querycamera );
            cout << "tracker tracked " << tracker->ntracked << " / " << tracker->nattempted << "\n";
            if ( !good ) break;
            if ( good ) {
                robustlsq.run( querycamera, 10 );
                SE3<> update = querycamera->node->pose * last_pose.inverse();
                if ( norm( update.ln() ) < 1e-6 ) break;
            }
        }
        tracktimer.click();
#endif
        return good;
    }
}
