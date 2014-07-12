
#include <match.h>

#include <Estimator/estimator.h>
#include <iostream>

#include <FeatureMatcher/featurematcher.h>

#include <opencv2/highgui.hpp>

namespace vrlt
{
    MatchThread::MatchThread( FeatureMatcher *_fm, Node *_node1, Node *_node2, double _threshold, bool _upright, bool _deleteFM )
    : fm( _fm ), node1( _node1 ), node2( _node2 ), threshold( _threshold ), upright( _upright ), deleteFM( _deleteFM )
    {
        
    }

    MatchThread::~MatchThread()
    {
		if ( deleteFM ) delete fm;
    }
    
    void MatchThread::run()
    {
        std::vector<Feature*> fm2_features;
        addFeatures( node2, false, fm2_features );

        if ( fm->features.empty() || fm2_features.empty() ) {
            return;
        }

//        findMatches( (*fm), fm2_features, matches );
        findUniqueMatches( (*fm), fm2_features, 0.8, matches );
        
        for ( int i = 0; i < matches.size(); i++ ) {
            PointPair point_pair;
            
            Feature *feature1 = matches[i]->feature1;
            Feature *feature2 = matches[i]->feature2;
            
            Eigen::Vector3d pt1 = feature1->unproject();
            Eigen::Vector3d pt2 = feature2->unproject();
            
            pt1 = feature1->camera->node->globalPose().inverse() * pt1;
            pt2 = feature2->camera->node->globalPose().inverse() * pt2;
            
            point_pair.first = pt1;
            point_pair.second = pt2;
            
            point_pairs.push_back( point_pair );
        }
        
//        PROSAC prosac;
//        prosac.num_trials = 100;
//        prosac.inlier_threshold = threshold;
//        prosac.min_num_inliers = point_pairs.size();
        
        std::vector<Estimator*> estimators( 100 );
        PreemptiveRANSAC ransac( 10 );
        ransac.inlier_threshold = threshold;
        
        if ( upright )
        {
            estimator = new UprightEssential;
            for ( size_t i = 0; i < estimators.size(); i++ ) estimators[i] = new UprightEssential;
        }
        else
        {
            estimator = new FivePointEssential;
            for ( size_t i = 0; i < estimators.size(); i++ ) estimators[i] = new FivePointEssential;
        }
        
//        ninliers = prosac.compute( point_pairs.begin(), point_pairs.end(), (*estimator), inliers );
        Estimator *best_estimator = NULL;
        ninliers = ransac.compute( point_pairs.begin(), point_pairs.end(), estimators, &best_estimator, inliers );
        
        if ( upright )
        {
            *(UprightEssential*)estimator = *(UprightEssential*)best_estimator;
        }
        else
        {
            *(FivePointEssential*)estimator = *(FivePointEssential*)best_estimator;
        }
        
        for ( size_t i = 0; i < estimators.size(); i++ ) delete estimators[i];
        estimators.clear();
    }
    
    void MatchThread::finish( Reconstruction &r )
    {
        std::cout << node1->name << "\t" << node2->name << "\t" << ninliers << " / " << point_pairs.size() << "\n";

        if ( ninliers < 50 ) {
            for ( int i = 0; i < inliers.size(); i++ ) {
                delete matches[i];
            }
            return;
        }
        
        PointPairList inlier_point_pairs;
        
        for ( int i = 0; i < inliers.size(); i++ ) {
            if ( inliers[i] == false ) {
                delete matches[i];
                continue;
            }
            inlier_point_pairs.push_back( point_pairs[i] );
            char name[256];
            sprintf( name, "match.%s.%s.%d", node1->name.c_str(), node2->name.c_str(), i );
            matches[i]->name = std::string(name);
            r.matches[ matches[i]->name ] = matches[i];
        }
        
        Pair *nodepair = new Pair;
        char name[256];
        sprintf( name, "pair.%s.%s", node1->name.c_str(), node2->name.c_str() );
        nodepair->name = name;
        nodepair->node1 = node1;
        nodepair->node2 = node2;
        if ( upright )
        {
            nodepair->pose = ((UprightEssential*)estimator)->getPose( inlier_point_pairs.begin(), inlier_point_pairs.end() );
        }
        else
        {
            nodepair->pose = ((FivePointEssential*)estimator)->getPose( inlier_point_pairs.begin(), inlier_point_pairs.end() );
        }

        nodepair->nmatches = (int)inlier_point_pairs.size();
        
        r.pairs[nodepair->name] = nodepair;
        
        delete estimator;
    }
}
