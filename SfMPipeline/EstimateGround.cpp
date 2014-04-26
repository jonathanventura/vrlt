
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <Estimator/estimator.h>

#include <opencv2/highgui.hpp>

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
    
    if ( r.nodes.find("root") == r.nodes.end() )
    {
        std::cout << "error: no root node\n";
        exit(1);
    }
    
    Node *root = (Node*)r.nodes["root"];
    
    std::vector<double> Yvalues;
    Yvalues.reserve( root->points.size() );
    
    for ( ElementList::iterator it = root->points.begin(); it != root->points.end(); it++ )
    {
        Point *point = (Point*)it->second;
        double Y = point->position[1]/point->position[3];
        if ( Y < 0 ) continue;  // negative Y is up
        Yvalues.push_back( Y );
    }
    
    std::sort( Yvalues.begin(), Yvalues.end() );
    
    double ground_height = Yvalues[Yvalues.size()*.8];
    
    Sophus::SE3d P;
    P.translation() << 0,-ground_height,0;
    transformPoints( root, P );
    
    XML::write( r, pathout );
}
