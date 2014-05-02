
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

using namespace vrlt;

void assignNormalsFromViewpoint( Node *rootnode )
{
    ElementList::iterator it;
    for ( it = rootnode->points.begin(); it != rootnode->points.end(); it++ )
    {
        Point *point = (Point *)it->second;
        Track *track = point->track;
        
        ElementList::iterator featureit;
        for ( featureit = track->features.begin(); featureit != track->features.end(); featureit++ )
        {
            Feature *feature = (Feature *)featureit->second;
            Camera *camera = feature->camera;
            Node *node = camera->node;
            
            if ( node->root() != rootnode ) continue;
            
            Eigen::Vector3d vec = node->globalRotation().inverse() * makeVector( 0., 0., -1. );
			point->normal = vec;
			break;
        }
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
    
    Node *root = (Node *)r.nodes["root"];
    
    assignNormalsFromViewpoint( root );
    
    XML::write( r, pathout );
}
