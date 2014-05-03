
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <ocam/ocam_functions.h>
#include <ocam/world2cam.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "extract_images.h"

#include <iostream>

using namespace vrlt;

#define NFACES 8

class Catadioptric : public ImageExtractor
{
    bool initialized;
    
    std::string pathocam;
    
    Sophus::SO3d rot_in;
    
    cv::Mat *catadioptricpts;
    Sophus::SO3d *Ry;
public:
    Catadioptric( cv::Size _sz, int _nfaces, Calibration *_calibration, std::string _pathocam, Sophus::SO3d _rot_in = Sophus::SO3d() )
    : ImageExtractor( _sz, _nfaces, _calibration ), initialized( false ), pathocam( _pathocam ), rot_in( _rot_in ),
    catadioptricpts( NULL ), Ry( NULL )
    {
    }
    
    ~Catadioptric()
    {
        delete [] catadioptricpts;
        delete [] Ry;
    }
    
    void create()
    {
        if ( initialized ) return;
        
        // load ocam parameters
        struct ocam_model ocam;
        int ret = get_ocam_model( &ocam, pathocam.c_str() );
        if ( ret != 0 )
        {
            fprintf( stderr, "error: could not read ocam file\n" );
            exit( 1 );
        }
        
        // create arrays
        catadioptricpts = new cv::Mat[nfaces];
        for ( int i = 0; i < nfaces; i++ ) catadioptricpts[i].create( sz, CV_64FC2 );
        Ry = new Sophus::SO3d[nfaces];
        
        // create intrinsic matrix
        Eigen::Matrix3d Kinv = Eigen::Matrix3d::Identity();
        Kinv(0,0) = 1./calibration->focal;
        Kinv(1,1) = 1./calibration->focal;
        Kinv(0,2) = -calibration->center[0]/calibration->focal;
        Kinv(1,2) = -calibration->center[1]/calibration->focal;
        
        // get 3D pts in camera reference frame
        cv::Mat pts( sz, CV_64FC3 );
        cv::Point2i loc( 0, 0 );
        for ( int y = 0; y < sz.height; y++ )
        {
            for ( int x = 0; x < sz.width; x++ )
            {
                Eigen::Vector3d X;
                X << x, y, 1;
                X = Kinv * X;
                pts.at<cv::Vec3d>(y,x) = cv::Vec3d(X[0],X[1],X[2]);
            }
        }
        
        // create rotations
        for ( int i = 0; i < nfaces; i++ )
        {
            Eigen::Vector3d rcatadioptric;
            rcatadioptric << 0.5 * M_PI, 0, 0;
            Sophus::SO3d Rcatadioptric = Sophus::SO3d::exp( rcatadioptric );
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
                    myworld2cam( catadioptricpts[i].at<cv::Vec2d>(y,x), Rcatadioptric * R[i] * X, &ocam );
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
                cv::Vec2d pos = catadioptricpts[face].at<cv::Vec2d>(y,x);
                float xmul = (float) input_image.size().width / 3840.f;
                float ymul = (float) input_image.size().height / 2160.f;
                cv::Point2f pt( pos[0]*xmul, pos[1]*ymul );
                output_image.at<cv::Vec3b>(y,x) = getColorSubpix( input_image, pt );
            }
        }
    }

    std::string getPath( const std::string &imagedir, int index )
    {
        char path[256];
        snprintf( path, 256, "%s/DSC%05d.JPG", imagedir.c_str(), index );
        return path;
    }
};

int main( int argc, char **argv )
{
    if ( argc != 7 && argc != 8 ) {
        fprintf( stderr, "usage: %s <ocam file> <image directory> <begin index> <step> <end index> [<file in>] <file out>\n", argv[0] );
        exit(1);
    }
    
    std::string pathocam = std::string(argv[1]);
    std::string imagedir = std::string(argv[2]);
    int begin_index = atoi(argv[3]);
    int step_index = atoi(argv[4]);
    int end_index = atoi(argv[5]);
    bool use_previous = ( argc == 8 );
    
    std::string pathout = std::string(argv[argc-1]);
    
    Reconstruction r;
    if ( use_previous ){
        XML::read( r, argv[argc-2] );
    }
    
    int base_height = 768;

    cv::Size sz( base_height * 2, base_height * .6 );
    
    Calibration *calibration = new Calibration;
    calibration->name = "calibration";
    calibration->focal = base_height / 2;
    calibration->center[0] = (sz.width-1.) / 2.;
    calibration->center[1] = (base_height/2.-1.);
    
    r.calibrations[ calibration->name ] = calibration;

    Catadioptric *catadioptric = NULL;
    
    if ( !use_previous )
    {
        std::cout << "creating catadioptric unwarper...\n";
        catadioptric = new Catadioptric( sz, NFACES, calibration, pathocam );
        catadioptric->create();
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
            
            Catadioptric *mycatadioptric = new Catadioptric( sz, NFACES, calibration, pathocam, node->pose.so3() );
            mythread = new ExtractThread( mycatadioptric, imagedir, i, true, "u" );
        } else {
            mythread = new ExtractThread( catadioptric, imagedir, i );
        }
        
        threads.push_back( mythread );
    }
    finishThreads( r, threads );
    
    if ( !use_previous ) delete catadioptric;
    
    XML::write( r, pathout );
}
