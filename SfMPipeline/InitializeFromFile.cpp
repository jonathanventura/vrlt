
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace vrlt;

bool readFile( const std::string &pathin, const std::string &imagedir, Reconstruction &r )
{
    FILE *f = fopen(pathin.c_str(),"r");
    
    char line[1024];
    
    // read header line
    fgets(line, 1024, f);
    
    int node_counter = 0;
    Node *node = NULL;
    
    // format: filename index fx fy cx cy R11 R12 R13 R21 R22 R23 R31 R32 R33
    while ( true )
    {
        int nread;
        
        char filename[1024];
        nread = fscanf(f,"%s",filename);
        if ( nread != 1 ) break;
        
        int index;
        nread = fscanf(f,"%d",&index);
        if ( nread != 1 ) return false;
        
        double fx, fy, cx, cy;
        nread = fscanf(f,"%lf %lf %lf %lf",&fx,&fy,&cx,&cy);
        if ( nread != 4 ) return false;
        
        double R11, R12, R13;
        double R21, R22, R23;
        double R31, R32, R33;
        nread = fscanf(f,"%lf %lf %lf",&R11,&R12,&R13);
        if ( nread != 3 ) return false;
        nread = fscanf(f,"%lf %lf %lf",&R21,&R22,&R23);
        if ( nread != 3 ) return false;
        nread = fscanf(f,"%lf %lf %lf",&R31,&R32,&R33);
        if ( nread != 3 ) return false;
        
        // make calibration (if necessary)
        std::stringstream calibration_name;
        calibration_name << "calibration" << index;
        Calibration *calibration = (Calibration*)r.calibrations[calibration_name.str()];
        if ( calibration == NULL )
        {
            calibration = new Calibration;
            calibration->name = calibration_name.str();
            calibration->focal = (fx+fy)/2.;
            calibration->center << cx,cy;
            r.calibrations[calibration->name] = calibration;
        }
        
        // make parent node (if necessary)
        if ( index == 0 )
        {
            std::stringstream node_name;
            node_name << "node";
            node_name.fill('0');
            node_name.width(5);
            node_name << node_counter;
            node = new Node;
            node->name = node_name.str();
            r.nodes[node->name] = node;
            node_counter++;
        }
        
        // make child node
        std::stringstream child_node_name;
        child_node_name << node->name << "-" << index;
        Node *child_node = new Node;
        child_node->name = child_node_name.str();
        Eigen::Matrix3d R;
        R <<
        R11,R12,R13,
        R21,R22,R23,
        R31,R32,R33;
        child_node->pose.so3() = Sophus::SO3d(R);
        node->children[child_node->name] = child_node;
        child_node->parent = node;
        
        // make camera
        std::stringstream camera_name;
        camera_name << node->name << "-camera" << index;
        Camera *camera = new Camera;
        camera->name = camera_name.str();
        r.cameras[camera->name] = camera;
        camera->calibration = calibration;
        std::stringstream camera_path;
        camera_path << imagedir << "/" << filename;
        camera->path = camera_path.str();
        camera->node = child_node;
        child_node->camera = camera;
    }
    
    fclose( f );
    
    return true;
}

int main( int argc, char **argv )
{
    if ( argc != 4 ) {
        fprintf( stderr, "usage: %s <file in> <image directory> <file out>\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string imagedir = std::string(argv[2]);
    std::string pathout = std::string(argv[3]);
    
    Reconstruction r;
    
    readFile( pathin, imagedir, r );
    
    XML::write( r, pathout );
}
