
#include "extract_images.h"

#include <iostream>

namespace vrlt
{
    ImageExtractor::ImageExtractor( cv::Size &_sz, int _nfaces, Calibration *_calibration )
    : sz( _sz ), nfaces( _nfaces ), calibration( _calibration )
    {
        R = new Sophus::SO3d[nfaces];
    }

    ImageExtractor::~ImageExtractor()
    {
        delete [] R;
    }

    
    
    ExtractThread::ExtractThread( ImageExtractor *_extractor, std::string _imagedir, int _index, bool _delete_extractor, std::string _prefix )
    : extractor( _extractor ), imagedir( _imagedir ), index( _index ), prefix( _prefix ), delete_extractor( _delete_extractor )
    {
        char name[256];
        if ( imagedir == "." )
            sprintf( name, "node%05d", index );
        else
            sprintf( name, "%s.node%05d", imagedir.c_str(), index );
        node = new Node;
        node->name = name;
    }
    
    ExtractThread::~ExtractThread()
    {
        if ( delete_extractor ) delete extractor;
    }
    
    void ExtractThread::run()
    {
        extractor->create();
        
        std::string path = extractor->getPath( imagedir, index );
        
        cv::Mat input_image;
        
        input_image = cv::imread( path.c_str(), cv::IMREAD_COLOR );
        
        for ( int i = 0; i < extractor->getNumFaces(); i++ )
        {
            char path[256];
            sprintf( path, "%s/%sface%d.image%05d.jpg", imagedir.c_str(), prefix.c_str(), i, index );
            
            cv::Mat output_image( extractor->getSize(), CV_8UC3, cv::Scalar(0,0,0) );
            extractor->extractImage( input_image, i, output_image );//, xmul, ymul );
            cv::imwrite( path, output_image );
            
            char name[256];
            
            Camera *camera = new Camera;
            if ( imagedir == "." )
                sprintf( name, "image%05d.%d", index, i );
            else
                sprintf( name, "%s.image%05d.%d", imagedir.c_str(), index, i );
            camera->name = name;
            camera->calibration = extractor->getCalibration();
            camera->path = path;
            
            Node *child = new Node;
            if ( imagedir == "." )
                sprintf( name, "node%05d.%d", index, i );
            else
            sprintf( name, "%s.node%05d.%d", imagedir.c_str(), index, i );
            child->name = name;
            child->parent = node;
            node->children[name] = child;
            child->camera = camera;
            camera->node = child;
            
            child->pose.so3() = extractor->getRotation(i).inverse();
        }
        
        if ( delete_extractor )
        {
            delete extractor;
            extractor = NULL;
        }
    }
    
    void ExtractThread::finish( Reconstruction &r )
    {
        if ( node == NULL ) return;
        
        ElementList::iterator it;
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node*)it->second;
            r.cameras[ child->camera->name ] = child->camera;
        }
        
        r.nodes[ node->name ] = node;
        std::cout << node->name << "\n";
    }
    
}
