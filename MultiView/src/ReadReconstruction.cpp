
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <iostream>

int main( int argc, char **argv )
{
    if ( argc != 2 )
    {
        fprintf(stdout,"usage: %s <directory>\n",argv[0]);
        exit(0);
    }
    
    std::string directory( argv[1] );

    vrlt::Reconstruction r;
    r.pathPrefix = directory;

    std::stringstream xmlfile;
    xmlfile << directory << "/reconstruction.xml";

    vrlt::XML::read( r, xmlfile.str() );

    vrlt::Node *root = (vrlt::Node*)r.nodes.begin()->second;

    vrlt::ElementList::iterator it;

    // How to iterate through all 3D points:
    for ( it = root->points.begin(); it != root->points.end(); it++ )
    {
        vrlt::Point *point = (vrlt::Point*)it->second;
        
        // Point position (4d homogeneous vector)
        Eigen::Vector4d position = point->position;
        
        // Features which observe this point:
        vrlt::Track *track = point->track;
        vrlt::ElementList::iterator featit;
        for ( featit = track->features.begin(); featit != track->features.end(); featit++ )
        {
            vrlt::Feature *feature = (vrlt::Feature*)featit->second;

            // Camera for this feature.
            vrlt::Camera *camera = feature->camera;

            // Location of the feature in the image. 
            Eigen::Vector2d location = feature->location;
            
            // Descriptor for the feature (128 uchar).
            const unsigned char *descriptor = feature->descriptor;
        }
    }

    // How to iterate through all cameras:
    for ( it = r.cameras.begin(); it != r.cameras.end(); it++ )
    {
        vrlt::Camera *camera = (vrlt::Camera*)it->second;
        if ( !camera->node ) continue;
        
        // Camera pose
        Sophus::SE3d pose = camera->node->globalPose();
        
        // Features for this camera:
        vrlt::ElementList::iterator featit;
        for ( featit = camera->features.begin(); featit != camera->features.end(); featit++ )
        {
            vrlt::Feature *feature = (vrlt::Feature*)featit->second;
            if ( !feature->track ) continue;

            vrlt::Track *track = feature->track;
            if ( !track->point ) continue;

            vrlt::Point *point = track->point;

            // Position of 3D point for this feature
            Eigen::Vector4d position = point->position;

            // Location of the feature in the image. 
            Eigen::Vector2d location = feature->location;
            
            // Descriptor for the feature (128 uchar).
            const unsigned char *descriptor = feature->descriptor;
        }
    }
    
    return 0;
}
