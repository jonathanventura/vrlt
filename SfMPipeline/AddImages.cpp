
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <Estimator/estimator.h>

#include <BundleAdjustment/bundle.h>
#include <BundleAdjustment/outliers.h>

#include <opencv2/highgui.hpp>

#include <iostream>

using namespace vrlt;

struct Candidate
{
    Node *node;
    int ntracks;
};

struct SortCandidates
{
    bool operator()( const Candidate &a, const Candidate &b ) { return a.ntracks > b.ntracks; }
};

struct MyMatch
{
    double score;
    Feature *feature;
    Point *point;
};

struct SortMyMatches
{
    bool operator()( const MyMatch &a, const MyMatch &b ) { return a.score < b.score; }
};

struct ImageAdder
{
    double thresh;
    Node *rootnode;
    ElementList oldNodes;
    ElementList oldPoints;
    ElementList newNodes;
    ElementList newPoints;
    
    ImageAdder( Reconstruction &_r ) : r( _r )
    {
        rootnode = (Node*)r.nodes["root"];
        
        ElementList::iterator camerait = r.cameras.begin();
        Camera *camera = (Camera*)camerait->second;
        cv::Mat image = cv::imread( camera->path, cv::IMREAD_GRAYSCALE );
        int width = image.size().width;
        switch ( camera->calibration->type )
        {
            case Calibration::Perspective:
                thresh = 4.f / camera->calibration->focal;
                break;
                
            case Calibration::Spherical:
            case Calibration::Cylindrical:
                thresh = .01;
                break;
        }
    }
    
    void run()
    {
        int nadded;
        do {
            nadded = addImages();
        } while ( nadded > 0 );
    }
    
    void findFeatures( Node *node, bool triangulated, ElementList &features )
    {
        _findFeatures( node, node, triangulated, features );
    }
    
    void _findFeatures( Node *root, Node *node, bool triangulated, ElementList &features )
    {
        Camera *camera = node->camera;
        if ( camera != NULL )
        {
            ElementList::iterator featureit;
            for ( featureit = camera->features.begin(); featureit != camera->features.end(); featureit++ )
            {
                Feature *feature = (Feature*)featureit->second;
                Track *track = feature->track;
                if ( track == NULL ) continue;
                Point *point = track->point;
                if ( triangulated && point == NULL ) continue;
                if ( !triangulated && point != NULL ) continue;

                features[feature->name] = feature;
            }
        }
        
        ElementList::iterator childit;
        for ( childit = node->children.begin(); childit != node->children.end(); childit++ )
        {
            Node *child = (Node *)childit->second;
            _findFeatures( node, child, triangulated, features );
        }
    }
    
    void findCandidates( std::vector<Candidate> &candidates )
    {
        ElementList::iterator it;
        for ( it = r.nodes.begin(); it != r.nodes.end(); it++ )
        {
            Node *node = (Node *)it->second;
            
            // see if it has already been added
            if ( node->root() == rootnode ) continue;
            
            // count triangulated features
            ElementList features;
            findFeatures( node, true, features );
            int count = (int)features.size();
            
            if ( count == 0 ) continue;
            
            Candidate candidate;
            candidate.node = node;
            candidate.ntracks = count;
            candidates.push_back( candidate );
            
            if ( r.upright ) break;
        }
    }
    
    void get2d3d( Node *node, std::vector<MyMatch> &mymatches )
    {
        ElementList features;
        findFeatures( node, true, features );
        
        ElementList::iterator featureit;
        for ( featureit = features.begin(); featureit != features.end(); featureit++ )
        {
            Feature *feature = (Feature *)featureit->second;
            Track *track = feature->track;
            Point *point = track->point;
            
            // make sure this is not a new point
            if ( newPoints.count( point->name ) > 0 ) continue;
            
            // find matches in this track, choose best score among them
            MyMatch mymatch;
            mymatch.point = point;
            mymatch.feature = feature;
            mymatch.score = INFINITY;
            ElementList::iterator matchit;
            for ( matchit = feature->matches.begin(); matchit != feature->matches.end(); matchit++ )
            {
                Match *match = (Match*)matchit->second;
                Feature *matched_feature;
                
                if ( match->feature1 == feature ) {
                    matched_feature = match->feature2;
                } else {
                    matched_feature = match->feature1;
                }
                
                if ( track->features.count( matched_feature->name ) == 0 ) continue;
                if ( match->score < mymatch.score ) {
                    mymatch.score = match->score;
                }
            }
            
            mymatches.push_back( mymatch );
        }
    }
    
    int resection( Node *node, std::vector<MyMatch> &mymatches, Sophus::SE3d &pose, std::vector<bool> &inliers )
    {
        // sort by match score
        std::sort( mymatches.begin(), mymatches.end(), SortMyMatches() );
        
        // get point pairs
        PointPairList point_pairs;
        for ( int j = 0; j < mymatches.size(); j++ )
        {
            PointPair point_pair;
            point_pair.first = project( mymatches[j].point->position );
            point_pair.second = mymatches[j].feature->globalUnproject();
            point_pairs.push_back( point_pair );
        }
        
        // estimate pose
        Estimator *estimator;
        
        estimator = new ThreePointPose;

        PROSAC prosac;
        prosac.num_trials = 2000;
        prosac.inlier_threshold = thresh;
        prosac.min_num_inliers = (int) point_pairs.size();
        
        int ninliers = prosac.compute( point_pairs.begin(), point_pairs.end(), (*estimator), inliers );
        
        std::cout << node->name << " ninliers: " << ninliers << " / " << point_pairs.size() << "\n";
        
        pose = ((ThreePointPose*)estimator)->pose;
        
        std::cout << "pose:\n";
        std::cout << pose.log() << "\n";
        std::cout << "norm: " << (pose.so3().inverse() * pose.translation()).norm() << "\n";
        delete estimator;
        
        return ninliers;
    }
    
    void addNewPoints( Node *node )
    {
        int count = 0;
        
        // find tracks to triangulate
        ElementList features;
        findFeatures( node, false, features );
        std::cout << features.size() << " possible features to triangulate\n";
        
        std::vector<Track*> tracksToRemove;
        std::vector<Feature*> featuresToRemove;

        ElementList::iterator featureit;
        for ( featureit = features.begin(); featureit != features.end(); featureit++ )
        {
            Feature *feature = (Feature *)featureit->second;
            Track *track = feature->track;
            if ( track->point != NULL ) continue;
            
            // find other cameras with known pose which view this point
            // choose one with largest angle
            Feature *best_feature = NULL;
            double best_angle = 0;
            ElementList::iterator it;
            for ( it = track->features.begin(); it != track->features.end(); it++ )
            {
                Feature *other_feature = (Feature*)it->second;
                if ( feature == other_feature ) continue;
                
                // make sure it's not the same camera!
                if ( feature->camera == other_feature->camera ) continue;
                
                // make sure it's not the same parent as well
                if ( feature->camera->node->parent != rootnode ) {
                    if ( feature->camera->node->parent == other_feature->camera->node->parent ) continue;
                }

                // make sure camera has pose
                Node *other_node = other_feature->camera->node;
                if ( other_node->root() != rootnode ) continue;
                
                // get rays and angle between them
                Eigen::Vector3d pt1 = feature->globalUnproject();
                Eigen::Vector3d pt2 = other_feature->globalUnproject();
                
                double angle = acos( pt1.dot( pt2 ) / pt1.norm() / pt2.norm() ) * 180. / M_PI;
                if ( angle < 0.5 ) continue;
                if ( angle > 90. ) continue;
                if ( angle > best_angle )
                {
                    best_angle = angle;
                    best_feature = other_feature;
                }
            }
            
            if ( best_feature == NULL ) continue;
            
            // triangulate point
            Eigen::Vector4d pt = triangulate( feature, best_feature );
            double inv_dist = fabs(pt[3]) / pt.head(3).norm();
            
            // add point
            Point *point = new Point;
            char name[256];
            sprintf( name, "point.%s.%d", node->name.c_str(), count++ );
            point->name = std::string(name);
            point->position = pt;
            
            r.link( rootnode, point );
            r.link( track, point );

            newPoints[point->name] = point;
            
            // check here for any repeated cameras in the track
            std::map<Camera*,int> cameraCounts;
            for ( it = track->features.begin(); it != track->features.end(); it++ )
            {
                Feature *feature = (Feature*)it->second;
                Camera *camera = feature->camera;
                if ( cameraCounts.count(camera) == 0 ) cameraCounts[camera] = 1;
                else cameraCounts[camera]++;
            }
            
            // for a repeated camera, choose the observation with the lowest reprojection error
            std::map<Camera*,int>::iterator mapit;
            for ( mapit = cameraCounts.begin(); mapit != cameraCounts.end(); mapit++ )
            {
                Camera *camera = mapit->first;
                int count = mapit->second;
                if ( count > 1 )
                {
                    std::cout << "*** FOUND A REPEATED FEATURE (" << count << ") ***\n";
                    double bestReproj = INFINITY;
                    Feature *feature_to_keep = NULL;
                    std::vector<Feature*> feature_list;
                    for ( it = track->features.begin(); it != track->features.end(); it++ )
                    {
                        Feature *feature = (Feature*)it->second;
                        if ( feature->camera != camera ) continue;
                        feature_list.push_back( feature );
                        Eigen::Vector2d diff = computeReprojError( feature );
                        double reproj = diff.dot(diff);
                        if ( reproj < bestReproj )
                        {
                            bestReproj = reproj;
                            feature_to_keep = feature;
                        }   
                    }
                    std::cout << "*** BEST REPROJ: " << sqrt(bestReproj) << " ***\n";
                    for ( int i = 0; i < feature_list.size(); i++ )
                    {
                        if ( feature_list[i] != feature_to_keep )
                        {
                            featuresToRemove.push_back( feature_list[i] );
                        }
                    }
                }
            }
        }
        
        for ( int i = 0; i < featuresToRemove.size(); i++ )
        {
            r.unlink( featuresToRemove[i]->track, featuresToRemove[i] );
        }
        
        for ( int i = 0; i < tracksToRemove.size(); i++ )
        {
            r.remove( tracksToRemove[i] );
        }
    }
    
    void retriangulate()
    {
        ElementList::iterator it;
        for ( it = rootnode->points.begin(); it != rootnode->points.end(); it++ )
        {
            Point *point = (Point *)it->second;
            Track *track = point->track;
            triangulate( rootnode, track );
        }
    }
    
    int addImages()
    {
        // find potential images to add
        // by counting their triangulated tracks
        std::vector<Candidate> candidates;
        findCandidates( candidates );
        
        // sort by number of tracks
        // (in phototourism, sorts by homography inlier ratio)
        std::sort( candidates.begin(), candidates.end(), SortCandidates() );
        
        for ( int i = 0; i < candidates.size(); i++ )
            std::cout << candidates[i].node->name << "\t" << candidates[i].ntracks << "\n";
        
        if ( candidates.empty() ) return 0;
        
        // threshold at .75 * max # matches
        int maxcount = candidates[0].ntracks;
        int countthresh = maxcount * 3 / 4;
        
        // collect nodes and points which we already have
        oldPoints = rootnode->points;
        oldNodes.clear();
        ElementList::iterator it;
        for ( it = rootnode->children.begin(); it != rootnode->children.end(); it++ )
        {
            Node *node = (Node*) it->second;
            oldNodes[node->name] = node;
        }
        
        // now try to add new nodes
        newNodes.clear();
        newPoints.clear();
        
        for ( int i = 0; i < candidates.size(); i++ )
        {
            // check # tracks
            if ( candidates[i].ntracks < countthresh ) {
                if ( !newNodes.empty() ) {
                    break;
                } else {
                    maxcount = candidates[i].ntracks;
                    countthresh = maxcount * 3 / 4;
                }
            }
            
            // get 2d-3d correspondences
            Node *node = candidates[i].node;
            std::vector<MyMatch> mymatches;
            get2d3d( node, mymatches );

            Sophus::SE3d pose;
            std::vector<bool> inliers;
            int ninliers = resection( node, mymatches, pose, inliers );
            if ( ninliers < 16 ) continue;
            
            // remove outlier feature from tracks
            for ( int j = 0; j < mymatches.size(); j++ )
            {
                if ( inliers[j] ) continue;
                Feature *feature = mymatches[j].feature;
                Track *track = feature->track;
                r.unlink( track, feature );
            }
            
            // add node
            r.attach( node, rootnode, pose );
            newNodes[node->name] = node;
        }
        
        // bundle to improve camera locations
		if ( !newNodes.empty() ) {
            Bundle *bundle = new Bundle( rootnode, oldNodes, oldPoints, false );
            bundle->run();
            delete bundle;
        }
        
        for ( it = newNodes.begin(); it != newNodes.end(); it++ )
        {
            Node *node = (Node*)it->second;
            
            // add points
            addNewPoints( node );
        }
        
        std::cout << "added " << newPoints.size() << " points\n";
        
        removeOutliers( &r, rootnode );
        
        if ( newNodes.empty() ) return 0;
        
		if ( !newNodes.empty() || !newPoints.empty() ) {
            Bundle *bundle = new Bundle( rootnode, oldNodes, oldPoints, r.upright );
            bundle->run();
            delete bundle;
        }
            
        runBundle( &r );
        
        return (int)newNodes.size();
    }
    
    Reconstruction &r;
};

void clearNode( Reconstruction &r, Node *node )
{
    if ( node->camera != NULL )
    {
        Camera *camera = node->camera;
        
        ElementList::iterator it;
        for ( it = camera->features.begin(); it != camera->features.end(); it++ )
        {
            Feature *feature = (Feature*)it->second;
            Track *track = feature->track;
            
            if ( track != NULL )
            {
                // remove link from point to track
                Point *point = track->point;
                if ( point != NULL )
                {
                    point->track = NULL;
                }
                
                // remove links from features to track
                ElementList::iterator it2;
                for ( it2 = track->features.begin(); it2 != track->features.end(); it2++ )
                {
                    Feature *feature2 = (Feature*)it2->second;
                    feature2->track = NULL;
                }
                
                // remove track from list
                r.tracks.erase( track->name );
            }
            
            // erase any matches with this feature
            ElementList::iterator matchit;
            for ( matchit = feature->matches.begin(); matchit != feature->matches.end(); matchit++ )
            {
                Match *match = (Match*)matchit->second;
                if ( r.matches.count( match->name ) == 0 ) continue;
                r.matches.erase( match->name );

            }
            
            // remove feature from list
            r.features.erase( feature->name );
        }
        
    }
    
    ElementList::iterator it;
    for ( it = node->children.begin(); it != node->children.end(); it++ )
    {
        Node *child = (Node*)it->second;
        clearNode( r, child );
    }
}

int main( int argc, char **argv )
{
    if ( argc != 3 ) {
        fprintf( stderr, "usage: %s <file in> <file out>\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string pathout = std::string(argv[2]);
    
    Reconstruction r;
    XML::read( r, pathin );
    
    ImageAdder imageAdder( r );
    imageAdder.run();

    Node *root = (Node*)r.nodes["root"];
    r.clearPairs( root );
    XML::write( r, pathout );

    // get remaining
    clearNode( r, root );
    r.nodes.erase( "root" );
    XML::write( r, "remaining.xml" );
}
