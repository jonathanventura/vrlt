
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <Estimator/estimator.h>

#include <BundleAdjustment/bundle.h>
#include <BundleAdjustment/outliers.h>

#include <opencv2/highgui.hpp>

#include <iostream>
#include <cmath>

using namespace vrlt;

int width;
double focal;

struct ImagePair
{
    int index1;
    int index2;
    size_t matchCount;
};

struct SortByCount
{
    bool operator()( const ImagePair &a, const ImagePair &b ) { return a.matchCount > b.matchCount; }
};

struct MyMatch
{
    double score;
    Feature *feature1;
    Feature *feature2;
    Track *track;
    PointPair point_pair;
};

struct SortMyMatches
{
    bool operator()( const MyMatch &a, const MyMatch &b ) { return a.score < b.score; }
};

void getPairs( Reconstruction &r, std::vector<ImagePair> &image_pairs )
{

}

struct Tester
{
    Tester( Reconstruction &_r, Node *_node1, Node *_node2 ) : r( _r ), node1( _node1 ), node2( _node2 )
    {
        // iterate tracks to find point pairs
        ElementList::iterator trackit;
        for ( trackit = r.tracks.begin(); trackit != r.tracks.end(); trackit++ )
        {
            Track *track = (Track*) trackit->second;
            
            ElementList::iterator featureit1;
            for ( featureit1 = track->features.begin(); featureit1 != track->features.end(); featureit1++ )
            {
                Feature* feature1 = (Feature*)featureit1->second;
                Node *root1 = feature1->camera->node->root();
                if ( root1 != node1 ) continue;
                
                ElementList::iterator featureit2;
                for ( featureit2 = track->features.begin(); featureit2 != track->features.end(); featureit2++ )
                {
                    Feature* feature2 = (Feature*)featureit2->second;
                    Node *root2 = feature2->camera->node->root();
                    if ( root2 != node2 ) continue;
                    
                    MyMatch mymatch;
                    mymatch.score = INFINITY;
                    mymatch.feature1 = feature1;
                    mymatch.feature2 = feature2;
                    mymatch.track = track;
                    mymatches.push_back( mymatch );
                    
                    // find this match for score if it exists
                    ElementList::iterator matchit;
                    for ( matchit = feature1->matches.begin(); matchit != feature1->matches.end(); matchit++ )
                    {
                        Match *match = (Match *)matchit->second;
                        if ( match->feature1 == feature2 || match->feature2 == feature2 ) {
                            mymatch.score = match->score;
                            break;
                        }
                    }
                }
            }
        }
        
        std::sort( mymatches.begin(), mymatches.end(), SortMyMatches() );
        
        for ( int i = 0; i < mymatches.size(); i++ )
        {
            PointPair point_pair;
            point_pair.first = mymatches[i].feature1->globalUnproject();
            point_pair.second = mymatches[i].feature2->globalUnproject();
            
            point_pairs.push_back( point_pair );
        }
    }
    
    bool checkHomography()
    {
        std::cout << "testing " << node1->name << " and " << node2->name << "\n";
        
        double thresh = width * .004 / focal;
        
        Homography homography;
        PROSAC prosac;
        std::vector<bool> inliers;
        prosac.inlier_threshold = thresh;
        prosac.min_num_inliers = 20;
        
        int ninliers = prosac.compute( point_pairs.begin(), point_pairs.end(), homography, inliers );
        
        std::cout << "homography inliers: " << ninliers << " / " << point_pairs.size() << "\n";
        
        double ratio = (double) ninliers / point_pairs.size();
        return ( ratio < .5 );
    }
    
    Sophus::SE3d findPose()
    {
        ElementList::iterator it;
        for ( it = r.pairs.begin(); it != r.pairs.end(); it++ )
        {
            node_pair = (Pair *)it->second;

            if ( node_pair->node1 != node1 ) continue;
            if ( node_pair->node2 != node2 ) continue;
            return node_pair->pose;
        }
        std::cout << "could not find pair\n";

        PointPairList my_point_pairs;
        
        // find matches from tracks
        ElementList::iterator trackit;
        for ( trackit = r.tracks.begin(); trackit != r.tracks.end(); trackit++ )
        {
            Track *track = (Track*)trackit->second;
            
            Feature *feature1 = NULL;
            Feature *feature2 = NULL;
            
            ElementList::iterator featit;
            for ( featit = track->features.begin(); featit != track->features.end(); featit++ )
            {
                Feature *feature = (Feature*)featit->second;
                Node *feature_root = feature->camera->node->root();
                
                if ( feature_root == node1 ) feature1 = feature;
                if ( feature_root == node2 ) feature2 = feature;
            }

            if ( feature1 == NULL || feature2 == NULL ) continue;
            
            PointPair point_pair;
            point_pair.first = feature1->globalUnproject();
            point_pair.second = feature2->globalUnproject();
            my_point_pairs.push_back( point_pair );
        }
        

        FivePointEssential essential;

        double thresh = width * .006 / focal;
        PROSAC prosac;
        prosac.min_num_inliers = (int) my_point_pairs.size();
        prosac.inlier_threshold = thresh;
        std::vector<bool> inliers;
        int ninliers = prosac.compute( my_point_pairs.begin(), my_point_pairs.end(), essential, inliers );
        std::cout << ninliers << " / " << my_point_pairs.size() << " inliers\n";
        Sophus::SE3d pose = essential.getPose( my_point_pairs.begin(), my_point_pairs.end() );
        return pose;
    }
    
    void bootstrap()
    {
        // find relative pose
        Sophus::SE3d rel_pose = findPose();
        
        // make nodes for two cameras
        Node *rootnode = new Node;
        rootnode->name = "root";
        r.nodes[rootnode->name] = rootnode;
        
        node1->fixed = true;
        node2->scaleFixed = true;
        
        r.attach( node1, rootnode, Sophus::SE3d() );
        r.attach( node2, rootnode, rel_pose );
        
        std::vector<Track*> tracksToRemove;
        
        Sophus::SE3d node1pose = node1->globalPose();
        Sophus::SE3d node2pose = node2->globalPose();
        
        rel_pose = node2pose * node1pose.inverse();
        
        std::cout << node1pose.log().transpose() << "\n";
        std::cout << node2pose.log().transpose() << "\n";
        std::cout << mymatches.size() << " matches to triangulate\n";
        
        // triangulate points
        int count = 0;
        for ( int i = 0; i < mymatches.size(); i++ )
        {
            Track *track = mymatches[i].track;
            if ( track->point != NULL ) continue;
            
            // get rays and angle between them
            Eigen::Vector3d pt1 = point_pairs[i].first;
            Eigen::Vector3d pt2 = rel_pose.so3().inverse() * point_pairs[i].second;
            
            double angle = acos( pt1.dot( pt2 ) / pt1.norm() / pt2.norm() ) * 180. / M_PI;

            if ( angle < 0.5 ) continue;
            if ( angle > 90. ) continue;
            
            Eigen::Vector4d pt = triangulate( rel_pose, point_pairs[i] );
            
            if ( fabs(pt[3]) < 1e-10 ) continue;

            Point *point = new Point;
            point->position = pt;
            
            char name[256];
            sprintf( name, "point.%s.%d", node2->name.c_str(), count++ );
            point->name = std::string(name);
            
            r.link( track, point );
            
            r.link( rootnode, point );
        }
        
        removeOutliers( &r, rootnode, 4 );
    }
    
    Reconstruction &r;
    Node *node1;
    Node *node2;
    std::vector<MyMatch> mymatches;
    PointPairList point_pairs;
    Pair *node_pair;
};

struct PairFinder
{
    int minstep;
    
    PairFinder( Reconstruction &_r, int _minstep ) : minstep( _minstep ), r( _r )
    {
        // make count of matches between nodes
        int N = (int) r.nodes.size();
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> counts(N,N);
        for ( int i = 0; i < N; i++ )
        {
            for ( int j = 0; j < N; j++ )
            {
                counts(i,j) = 0;
            }
        }
        
        ElementList::iterator it;
        int n = 0;
        for ( it = r.nodes.begin(); it != r.nodes.end(); it++,n++ )
        {
            Node *node = (Node*) it->second;
            node2index[node->name] = n;
            index2node[n] = node->name;
        }
        
        // iterate tracks to count matches
        ElementList::iterator trackit;
        for ( trackit = r.tracks.begin(); trackit != r.tracks.end(); trackit++ )
        {
            Track *track = (Track*) trackit->second;
            
            ElementList::iterator featureit1;
            for ( featureit1 = track->features.begin(); featureit1 != track->features.end(); featureit1++ )
            {
                Feature* feature1 = (Feature*)featureit1->second;
                int index1 = node2index[feature1->camera->node->root()->name];
                
                ElementList::iterator featureit2 = featureit1;
                for ( featureit2++; featureit2 != track->features.end(); featureit2++ )
                {
                    Feature* feature2 = (Feature*)featureit2->second;
                    int index2 = node2index[feature2->camera->node->root()->name];
                    
                    if ( index1 < index2 ) {
                        counts(index1,index2)++;
                    } else if ( index1 > index2 ) {
                        counts(index2,index1)++;
                    }
                }
            }
        }
        
        for ( int i = 0; i < N; i++ ) {
            for ( int j = i+minstep; j < N; j++ ) {
                ImagePair image_pair;
                image_pair.index1 = i;
                image_pair.index2 = j;
                image_pair.matchCount = counts(i,j);
                image_pairs.push_back( image_pair );
            }
        }
        
        // order pairs by number of matches
        std::sort( image_pairs.begin(), image_pairs.end(), SortByCount() );
        
        std::vector<ImagePair>::iterator pairit;
        for ( pairit = image_pairs.begin(); pairit != image_pairs.end(); pairit++ )
        {
            Node *node1 = (Node*) r.nodes[ index2node[ pairit->index1 ] ];
            Node *node2 = (Node*) r.nodes[ index2node[ pairit->index2 ] ];
            
            if ( pairit->matchCount == 0 ) break;
            std::cout << node1->name << " " << node2->name << " " << pairit->matchCount << "\n";
        }
    }
    
    void clearNode( Node *node )
    {
        ElementList::iterator it;
        for ( it = node->points.begin(); it != node->points.end(); it++ )
        {
            delete it->second;
        }
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            clearNode( (Node*)it->second );
        }
        delete node;
    }
    
    void clear( Reconstruction *r_out )
    {
        ElementList::iterator it;
        for ( it = r_out->calibrations.begin(); it != r_out->calibrations.end(); it++ )
        {
            delete it->second;
        }
        for ( it = r_out->cameras.begin(); it != r_out->cameras.end(); it++ )
        {
            delete it->second;
        }
        for ( it = r_out->features.begin(); it != r_out->features.end(); it++ )
        {
            delete it->second;
        }
        for ( it = r_out->matches.begin(); it != r_out->matches.end(); it++ )
        {
            delete it->second;
        }
        for ( it = r_out->triplets.begin(); it != r_out->triplets.end(); it++ )
        {
            delete it->second;
        }
        for ( it = r_out->pairs.begin(); it != r_out->pairs.end(); it++ )
        {
            delete it->second;
        }
        for ( it = r_out->tracks.begin(); it != r_out->tracks.end(); it++ )
        {
            delete it->second;
        }
        for ( it = r_out->nodes.begin(); it != r_out->nodes.end(); it++ )
        {
            clearNode( (Node*)it->second );
        }
        delete r_out;
    }
    
    Reconstruction * run( std::string pathin )
    {
        Reconstruction *r_out = NULL;
        
        // find best pair that is not well-modeled by homography
        std::vector<ImagePair>::iterator pairit;
        for ( pairit = image_pairs.begin(); pairit != image_pairs.end(); pairit++ )
        {
            r_out = new Reconstruction;
            XML::read( *r_out, pathin );
            
            Node *node1 = (Node*) r_out->nodes[ index2node[ pairit->index1 ] ];
            Node *node2 = (Node*) r_out->nodes[ index2node[ pairit->index2 ] ];
            
            Tester tester( *r_out, node1, node2 );
            
            bool good;
            
            tester.bootstrap();
            
            Node *root = (Node*)r_out->nodes["root"];
            
            std::cout << root->points.size() << " points\n";
            
            if ( root->points.size() < 100 ) {
                clear( r_out );
                node1->fixed = false;
                node2->scaleFixed = false;
                return NULL;
            }
            
            std::cout << node1->pose.translation().transpose() << "\n";
            std::cout << node2->pose.translation().transpose() << "\n";

            good = runBundle( r_out );
            if ( !good ) {
                std::cout << "bundle failed\n";
                clear( r_out );
                node1->fixed = false;
                node2->scaleFixed = false;
                return NULL;
            }
        
            if ( root->points.size() < 100 ) {
                clear( r_out );
                node1->fixed = false;
                node2->scaleFixed = false;
                return NULL;
            }
            
            break;
        }
   
        return r_out;
    }
    
    Reconstruction &r;
    std::map<std::string,int> node2index;
    std::map<int,std::string> index2node;
    std::vector<ImagePair> image_pairs;
};

int main( int argc, char **argv )
{
    if ( argc != 3 && argc != 4 && argc != 5 ) {
        fprintf( stderr, "usage: %s <file in> <file out> [ <minstep> [ | <node1> <node2>] ]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string pathout = std::string(argv[2]);
    
    Reconstruction r;
    XML::read( r, pathin );
    
    ElementList::iterator camerait = r.cameras.begin();
    Camera *camera = (Camera*)camerait->second;
    cv::Mat image = cv::imread( camera->path, cv::IMREAD_GRAYSCALE );
    width = image.size().width;
    focal = camera->calibration->focal;

    Reconstruction *r_out = &r;
    
    if ( argc < 5 ) {
        int minstep = 1;
        if ( argc == 4 ) minstep = atoi(argv[3]);
        PairFinder pairFinder( r, minstep );
        r_out = pairFinder.run( pathin );
    } else {
        std::string name1 = std::string(argv[3]);
        std::string name2 = std::string(argv[4]);
        
        if ( r.nodes.count( name1 ) == 0 ) {
            fprintf( stderr, "could not find node %s\n", name1.c_str() );
            exit(1);
        }

        if ( r.nodes.count( name2 ) == 0 ) {
            fprintf( stderr, "could not find node %s\n", name2.c_str() );
            exit(1);
        }

        Node *node1 = (Node*)r.nodes[name1];
        Node *node2 = (Node*)r.nodes[name2];
        
        Tester tester( r, node1, node2 );
        tester.bootstrap();

        runBundle( &r );
    }
    
    if ( r_out == NULL ) {
        std::cout << "error: bootstrap failed.\n";
        exit(1);
    }
    
    Node *root = (Node*)r_out->nodes["root"];
    r_out->clearPairs( root );
    
    XML::write( (*r_out), pathout );
}
