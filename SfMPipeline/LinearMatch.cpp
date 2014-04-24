#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <FeatureMatcher/featurematcher.h>
#include <FeatureMatcher/approxnn.h>
#include <Estimator/estimator.h>

#include <opencv2/highgui.hpp>

#include <iostream>

#include "threaded.h"
#include "match.h"

using namespace vrlt;

int main( int argc, char **argv )
{
    if ( argc != 4 && argc != 5 ) {
        fprintf( stderr, "usage: %s <file in> <file out> <numneighbors>\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string pathout = std::string(argv[2]);
    int maxnumneighbors = atoi(argv[3]);
    
    Reconstruction r;
    XML::read( r, pathin );
    
    // make a fake root node to contain all nodes
    Node *root = new Node;
    ElementList::iterator it;
    for ( it = r.nodes.begin(); it != r.nodes.end(); it++ )
    {
        Node *node = (Node*)it->second;
        root->children[node->name] = node;
    }
    
    XML::readDescriptors( r, root );

    ElementList::iterator camerait = r.cameras.begin();
    Camera *camera = (Camera*)camerait->second;
    cv::Mat image = cv::imread( camera->path, cv::IMREAD_GRAYSCALE );
    double threshold = 4.f / camera->calibration->focal;
    
    ElementList::iterator nodeit1;
    
    std::vector<ReconstructionThread*> threads;

    NN *nn = new ApproxNN;

    int nodeindex1 = 0;
    for ( nodeit1 = r.nodes.begin(); nodeit1 != r.nodes.end(); nodeit1++,nodeindex1++ ) {
        Node *node1 = (Node*) nodeit1->second;
        
        FeatureMatcher fm1( nn );
        fm1.init( node1 );
	
		std::cout << node1->name << "\n";

        int neighborcount = 0;
        ElementList::iterator nodeit2 = nodeit1;
        int nodeindex2 = nodeindex1;
        for ( nodeit2++; nodeit2 != r.nodes.end(); nodeit2++,nodeindex2++ ) {
            Node *node2 = (Node*) nodeit2->second;
            if ( neighborcount++ == maxnumneighbors ) break;
            if ( node1->root() == node2->root() ) continue;
			std::cout << "\t" << node2->name << "\n";
            
            MatchThread *thread = new MatchThread( &fm1, node1, node2, threshold, r.upright );

			threads.push_back( thread );
        }

		finishThreads( r, threads );
    }
    
    XML::write( r, pathout );
}
