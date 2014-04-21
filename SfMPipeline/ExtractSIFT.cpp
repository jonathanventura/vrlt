
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <FeatureExtraction/features.h>

#include <opencv2/highgui.hpp>

#include "threaded.h"

#include <iostream>

using namespace vrlt;

#define NTHREADS 8

int min_octave = 0;
const float peak_thresh = 0.5f;

class SIFTThread : public ReconstructionThread
{
public:
    
    SIFTThread( Camera *_camera, bool _upright = false, bool _make_points = false )
    : camera( _camera ), upright( _upright ), make_points( _make_points )
    {
        
    }
    
    void run()
    {
        cv::Mat color_image = cv::imread( camera->path, cv::IMREAD_COLOR );

        int nfeatures = extractSIFT( color_image, features, min_octave, false, peak_thresh );
        std::cout << "extracted " << nfeatures << " features for image " << camera->name << "\n";
        
        for ( int i = 0; i < features.size(); i++ ) {
            char name[256];
            sprintf( name, "%s.%04d", camera->name.c_str(), i );
            features[i]->name = std::string(name);
            features[i]->camera = camera;
            camera->features[ features[i]->name ] = features[i];
        }
    }
    
    void finish( Reconstruction &r )
    {
        for ( int i = 0; i < features.size(); i++ ) {
            r.features[ features[i]->name ] = features[i];
        }
        
        if ( make_points )
        {
            for ( int i = 0; i < features.size(); i++ ) {
                camera->features[ features[i]->name ] = features[i];
                
                char name[256];
                
                Track *track = new Track;
                sprintf( name, "track.%s.%d", camera->name.c_str(), i );
                track->name = name;
                track->features[ features[i]->name ] = features[i];
                r.tracks[ track->name ] = track;
                
                Point *point = new Point;
                sprintf( name, "point.%s.%d", camera->name.c_str(), i );
                point->name = name;
                point->position = unproject( features[i]->unproject() );
                point->normal << 0, 0, -1;
                track->point = point;
                point->track = track;
                camera->node->root()->points[ point->name ] = point;
            }
        }
        
        XML::writeFeatures( r, camera );
        XML::writeDescriptors( r, camera );
        XML::clearDescriptors( r, camera );
    }
    
    Camera *camera;
    std::vector<Feature*> features;
    bool upright;
    bool make_points;
};

int main( int argc, char **argv )
{
    if ( argc != 3 && argc != 4  && argc != 5 ) {
        fprintf( stderr, "usage: %s <file in> <file out> [<step>] [<min octave>]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string pathout = std::string(argv[2]);

    int step = 1;
    if ( argc > 3 ) step = atoi(argv[3]);
    if ( argc > 4 ) min_octave = atoi(argv[4]);
    
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
        if ( threads.size() == NTHREADS )
        {
            finishThreads( r, threads );
            std::cout << i << " completed\n";
        }
        
        Camera *camera = cameras[i];
        SIFTThread *thread = new SIFTThread( camera, false, false );
        threads.push_back( thread );
    }
    finishThreads( r, threads );
    
    XML::write( r, pathout );
}
