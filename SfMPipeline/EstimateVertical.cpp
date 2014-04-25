
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <Estimator/estimator.h>
#include "lines.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace vrlt;

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
    
    Camera *camera = (Camera*)r.cameras.begin()->second;
    camera->image = cv::imread( camera->path, cv::IMREAD_GRAYSCALE );
    Eigen::Vector2d imsize;
    imsize << camera->image.size().width, camera->image.size().height;
    double diagonal = imsize.norm();
    double min_length = .025 * diagonal;
    
    if ( r.nodes.find("root") != r.nodes.end() )
    {
        Node *root = (Node*)r.nodes["root"];
        Sophus::SO3d R = rectify( root, min_length );
        Sophus::SE3d P;
        P.so3() = R.inverse();
        root->pose = P * root->pose;
    }
    else
    {
        std::vector<Node*> oldNodes;
        std::vector<Node*> newNodes;
        
        ElementList::iterator nodeit;
        for ( nodeit = r.nodes.begin(); nodeit != r.nodes.end(); nodeit++ ) {
            Node *node = (Node*) nodeit->second;
            
            Sophus::SO3d R = rectify( node, min_length );
            node->pose.so3() = R.inverse();
        }
        
        r.upright = true;
    }
    
    XML::write( r, pathout );
}
