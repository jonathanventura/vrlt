
#ifndef MATCH_H

#include <threaded.h>

#include <MultiView/multiview.h>
#include <FeatureMatcher/featurematcher.h>

#include <iostream>

#include <Estimator/estimator.h>

namespace vrlt {
    class MatchThread : public ReconstructionThread
    {
        FeatureMatcher *fm;
        Node *node1;
        Node *node2;
        double threshold;
        PointPairList point_pairs;
        std::vector<Match*> matches;
        std::vector<bool> inliers;
        Estimator *estimator;
        int ninliers;
        bool upright;
		bool deleteFM;
        
    public:
        MatchThread( FeatureMatcher *_fm, Node *_node1, Node *_node2, double _threshold, bool _upright = false, bool _deleteFM = false );
		~MatchThread();
        
        void run();
        void finish( Reconstruction &r );
    };
}

#endif
