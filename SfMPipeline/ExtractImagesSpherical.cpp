
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "extract_images.h"

#include <iostream>

using namespace vrlt;

#define NFACES 8

class Spherical : public ImageExtractor
{
    bool initialized;
    
    cv::Size input_size;
    
    Sophus::SO3d rot_in;
    
    cv::Mat *sphericalpts;
    Sophus::SO3d *Ry;
public:
    Spherical( cv::Size _sz, int _nfaces, Calibration *_calibration, cv::Size _input_size, Sophus::SO3d _rot_in = Sophus::SO3d() )
    : ImageExtractor( _sz, _nfaces, _calibration ), initialized( false ), input_size( _input_size ), rot_in( _rot_in ),
    sphericalpts( NULL ), Ry( NULL )
    {
    }
    
    ~Spherical()
    {
        delete [] sphericalpts;
        delete [] Ry;
    }
    
    void create()
    {
        if ( initialized ) return;
        
        // get spherical parameters
        Calibration spherical_calibration;
        spherical_calibration.type = Calibration::Spherical;
        spherical_calibration.center << input_size.width*0.5 - 0.5, input_size.height*0.5 - 0.5;
        spherical_calibration.focal = input_size.height/M_PI;
        
        // create arrays
        sphericalpts = new cv::Mat[nfaces];
        for ( int i = 0; i < nfaces; i++ ) sphericalpts[i].create( sz, CV_64FC2 );
        Ry = new Sophus::SO3d[nfaces];
        
        // create intrinsic matrix
        Eigen::Matrix3d Kinv = Eigen::Matrix3d::Identity();
        Kinv(0,0) = 1./calibration->focal;
        Kinv(1,1) = 1./calibration->focal;
        Kinv(0,2) = -calibration->center[0]/calibration->focal;
        Kinv(1,2) = -calibration->center[1]/calibration->focal;
        
        // get 3D pts in camera reference frame
        cv::Mat pts( sz, CV_64FC3 );
        for ( int y = 0; y < sz.height; y++ )
        {
            for ( int x = 0; x < sz.width; x++ )
            {
                Eigen::Vector3d X;
                X << x, y, 1;
                X = Kinv * X;
                X.normalize();
                pts.at<cv::Vec3d>(y,x) = cv::Vec3d(X[0],X[1],X[2]);
            }
        }
        
        // create rotations
        for ( int i = 0; i < nfaces; i++ )
        {
            Eigen::Vector3d ry;
            ry << 0, i * 2. * M_PI / nfaces, 0;
            Ry[i] = Sophus::SO3d::exp(ry);
            R[i] = rot_in * Ry[i];
            
            // get points
            for ( int y = 0; y < sz.height; y++ )
            {
                for ( int x = 0; x < sz.width; x++ )
                {
                    Eigen::Vector3d X;
                    X << pts.at<cv::Vec3d>(y,x)[0], pts.at<cv::Vec3d>(y,x)[1], pts.at<cv::Vec3d>(y,x)[2];
                    X = R[i] * X;
                    Eigen::Vector2d Xproj = spherical_calibration.project3( X );
                    sphericalpts[i].at<cv::Vec2d>(y,x)[0] = Xproj[0];
                    sphericalpts[i].at<cv::Vec2d>(y,x)[1] = Xproj[1];
                }
            }
        }

        initialized = true;
    }
    
    void extractImage( cv::Mat &input_image, int face, cv::Mat &output_image ) const
    {
        for ( int y = 0; y < sz.height; y++ )
        {
            for ( int x = 0; x < sz.width; x++ )
            {
                cv::Vec2d pos = sphericalpts[face].at<cv::Vec2d>(y,x);
                cv::Point2f pt( pos[0], pos[1] );
                output_image.at<cv::Vec3b>(y,x) = getColorSubpix( input_image, pt );
            }
        }
    }
    
    void extractImage( cv::Mat &input_image, int face, cv::Mat &output_image, float xmul, float ymul ) const
    {
        for ( int y = 0; y < sz.height; y++ )
        {
            for ( int x = 0; x < sz.width; x++ )
            {
                cv::Vec2d pos = sphericalpts[face].at<cv::Vec2d>(y,x);
                pos[0] *= xmul;
                pos[1] *= ymul;
                cv::Point2f pt( pos[0], pos[1] );
                output_image.at<cv::Vec3b>(y,x) = getColorSubpix( input_image, pt );
            }
        }
    }
    
    std::string getPath( const std::string &imagedir, int index )
    {
        char path[256];
        snprintf( path, 256, "%s/R%07d.JPG", imagedir.c_str(), index );
        return path;
    }
};

int main( int argc, char **argv )
{
    if ( argc != 6 && argc != 7 ) {
        fprintf( stderr, "usage: %s <image directory> <begin index> <step> <end index> [<file in>] <file out>\n", argv[0] );
        exit(1);
    }
    
    std::string imagedir = std::string(argv[1]);
    int begin_index = atoi(argv[2]);
    int step_index = atoi(argv[3]);
    int end_index = atoi(argv[4]);
    bool use_previous = ( argc == 7 );
    
    std::string pathout = std::string(argv[argc-1]);
    
    Reconstruction r;
    if ( use_previous ){
        XML::read( r, argv[argc-2] );
    }
    
    int base_height = 768;
    
    cv::Size sz( base_height * .6, base_height );
    
    Calibration *calibration = new Calibration;
    calibration->name = "calibration";
    calibration->focal = base_height * .3;
    calibration->center[0] = sz.width * 0.5 - 0.5;
    calibration->center[1] = sz.height * 0.5 - 0.5;
    
    r.calibrations[ calibration->name ] = calibration;
    
    char path[256];
    sprintf( path, "%s/R%07d.JPG", imagedir.c_str(), begin_index );
    
    cv::Mat first_input_image = cv::imread( path, cv::IMREAD_COLOR );
    std::cout << "image size: " << first_input_image.size().width << " " << first_input_image.size().height << "\n";
    
    Spherical *spherical = NULL;
    
    if ( !use_previous )
    {
        std::cout << "creating spherical unwarper...\n";
        spherical = new Spherical( sz, NFACES, calibration, first_input_image.size() );
        spherical->create();
        std::cout << "done.\n";
    }
    
    std::vector<ReconstructionThread*> threads;
    for ( int i = begin_index; i <= end_index; i += step_index )
    {
        ExtractThread *mythread = NULL;
        
        if ( use_previous ) {
            char nodename[256];
            sprintf( nodename, "%s.node%05d", imagedir.c_str(), i );
            if ( r.nodes.count( nodename ) == 0 ) {
                fprintf( stderr, "could not find node %s\n", nodename );
                continue;
            }
            Node *node = (Node *)r.nodes[nodename];
            
            Spherical *myspherical = new Spherical( sz, NFACES, calibration, first_input_image.size(), node->pose.so3() );
            mythread = new ExtractThread( myspherical, imagedir, i, true, "u" );
        } else {
            mythread = new ExtractThread( spherical, imagedir, i );
        }
        
        threads.push_back( mythread );
    }
    finishThreads( r, threads );
    
    if ( !use_previous ) delete spherical;
    
    XML::write( r, pathout );
}
