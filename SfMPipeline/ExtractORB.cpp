
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <FeatureExtraction/features.h>

#include <opencv2/highgui.hpp>

#include "threaded.h"

#include <iostream>

using namespace vrlt;

class ORBThread : public ReconstructionThread
{
public:
    
    ORBThread( Camera *_camera, int _nfeatures )
    : camera( _camera ), nfeatures( _nfeatures )
    {
        
    }
    
    void run()
    {
        cv::Mat color_image = cv::imread( camera->path, cv::IMREAD_COLOR );

        extractORB( color_image, features, nfeatures );
        std::cout << "extracted " << features.size() << " features for image " << camera->name << "\n";
        
        for ( int i = 0; i < features.size(); i++ ) {
            char name[256];
            sprintf( name, "%s.%04d", camera->name.c_str(), i );
            features[i]->name = std::string(name);
            features[i]->camera = camera;
            camera->features[ features[i]->name ] = features[i];
        }
        
        color_image.resize(0);
    }
    
    void finish( Reconstruction &r )
    {
        for ( int i = 0; i < features.size(); i++ ) {
            r.features[ features[i]->name ] = features[i];
        }
        
        XML::writeFeatures( r, camera );
        XML::writeDescriptors( r, camera );
        XML::clearDescriptors( r, camera );
    }
    
    Camera *camera;
    int nfeatures;
    std::vector<Feature*> features;
};

int main( int argc, char **argv )
{
    if ( argc != 3 && argc != 4 && argc != 5 ) {
        fprintf( stderr, "usage: %s <file in> <file out> [<step>] [<nfeatures]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string pathout = std::string(argv[2]);

    int step = 1;
    int nfeatures = 500;
    if ( argc > 3 ) step = atoi(argv[3]);
    if ( argc > 4 ) nfeatures = atoi(argv[4]);
    
    Reconstruction r;
    XML::read( r, pathin, false );
    
    std::vector<Camera*> cameras;
    std::vector<Camera*> cameras_to_remove;
    
    int count = 0;
    
    // get cameras and images
    ElementList::iterator camit;
    for ( camit = r.cameras.begin(); camit != r.cameras.end(); camit++,count++ ) {
        Camera *camera = (Camera*) camit->second;

        if ( count % step != 0 ) {
            cameras_to_remove.push_back( camera );
            continue;
        }
        
        cameras.push_back( camera );
        
        if ( camera->node == NULL )
        {
            Node *node = new Node;
            char name[256];
            sprintf( name, "node.%s", camera->name.c_str() );
            node->name = name;
            node->camera = camera;
            camera->node = node;
            r.nodes[name] = node;
        }
    }
    
    for ( int i = 0; i < cameras_to_remove.size(); i++ )
    {
        r.cameras.erase( cameras_to_remove[i]->name );
        if ( cameras_to_remove[i]->node ) {
            r.nodes.erase( cameras_to_remove[i]->node->name );
        }
    }
    
    std::vector<ReconstructionThread*> threads;
    for ( int i = 0; i < cameras.size(); i++ )
    {
        Camera *camera = cameras[i];
        ORBThread *thread = new ORBThread( camera, nfeatures );
        threads.push_back( thread );
    }
    finishThreads( r, threads );
    
    XML::write( r, pathout );
}
