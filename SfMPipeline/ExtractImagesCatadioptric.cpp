
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <ocam/ocam_functions.h>
#include <ocam/world2cam.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "threaded.h"

#include <iostream>

using namespace vrlt;

#define NTHREADS 16
#define NFACES 8

static cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt)
{
    assert(!img.empty());
    assert(img.channels() == 3);
    
    int x = (int)pt.x;
    int y = (int)pt.y;
    
    int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
    int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
    int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
    int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);
    
    float a = pt.x - (float)x;
    float c = pt.y - (float)y;
    
    uchar b = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[0] * a) * (1.f - c)
                             + (img.at<cv::Vec3b>(y1, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[0] * a) * c);
    uchar g = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[1] * a) * (1.f - c)
                             + (img.at<cv::Vec3b>(y1, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[1] * a) * c);
    uchar r = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[2] * a) * (1.f - c)
                             + (img.at<cv::Vec3b>(y1, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[2] * a) * c);
    
    return cv::Vec3b(b, g, r);
}

class Catadioptric
{
public:
    Catadioptric( std::string pathocam, Calibration *_calibration, cv::Size _sz, int _nfaces, Sophus::SO3d _rot_in = Sophus::SO3d() )
    : sz( _sz ), nfaces( _nfaces ), rot_in( _rot_in ), calibration( _calibration )
    {
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
        R = new Sophus::SO3d[nfaces];
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
    }
    
    void extractImage( cv::Mat &catadioptric_image, int face, cv::Mat &output_image ) const
    {
        for ( int y = 0; y < sz.height; y++ )
        {
            for ( int x = 0; x < sz.width; x++ )
            {
                cv::Vec2d pos = catadioptricpts[face].at<cv::Vec2d>(y,x);
                cv::Point2f pt( pos[0], pos[1] );
                output_image.at<cv::Vec3b>(y,x) = getColorSubpix( catadioptric_image, pt );
            }
        }
    }

    void extractImage( cv::Mat &catadioptric_image, int face, cv::Mat &output_image, float xmul, float ymul ) const
    {
        for ( int y = 0; y < sz.height; y++ )
        {
            for ( int x = 0; x < sz.width; x++ )
            {
                cv::Vec2d pos = catadioptricpts[face].at<cv::Vec2d>(y,x);
                pos[0] *= xmul;
                pos[1] *= ymul;
                cv::Point2f pt( pos[0], pos[1] );
                output_image.at<cv::Vec3b>(y,x) = getColorSubpix( catadioptric_image, pt );
            }
        }
    }

    ~Catadioptric()
    {
        delete [] catadioptricpts;
        delete [] R;
        delete [] Ry;
    }
    
    cv::Size sz;
    int nfaces;
    
    cv::Mat *catadioptricpts;
    Sophus::SO3d *R;
    Sophus::SO3d *Ry;
    Sophus::SO3d rot_in;
    
    Calibration *calibration;
};

class ExtractThread : public ReconstructionThread
{
public:
    ExtractThread( const Catadioptric *_catadioptric, std::string _imagedir, int _index, bool _delete_catadioptric = false, std::string _prefix = "" )
    : catadioptric( _catadioptric ), imagedir( _imagedir ), index( _index ), prefix( _prefix ), delete_catadioptric( _delete_catadioptric )
    {
        char name[256];
        sprintf( name, "%s.node%05d", imagedir.c_str(), index );
        node = new Node;
        node->name = name;
    }
    
    ~ExtractThread()
    {
        if ( delete_catadioptric ) delete catadioptric;
    }
    
    void run()
    {   
        char path[256];
        sprintf( path, "%s/DSC%05d.JPG", imagedir.c_str(), index );

        cv::Mat catadioptric_image;
        
        catadioptric_image = cv::imread( path, cv::IMREAD_COLOR );
        
        float xmul = (float) catadioptric_image.size().width / 3840.f;
        float ymul = (float) catadioptric_image.size().height / 2160.f;

        for ( int i = 0; i < catadioptric->nfaces; i++ )
        {
            sprintf( path, "%s/%sface%d.image%05d.jpg", imagedir.c_str(), prefix.c_str(), i, index );
            
            cv::Mat output_image( catadioptric->sz, CV_8UC3, cv::Scalar(0,0,0) );
            catadioptric->extractImage( catadioptric_image, i, output_image, xmul, ymul );
            cv::imwrite( path, output_image );
            
            char name[256];
            
            Camera *camera = new Camera;
            sprintf( name, "%s.image%05d.%d", imagedir.c_str(), index, i );
            camera->name = name;
            camera->calibration = catadioptric->calibration;
            camera->path = path;
            
            Node *child = new Node;
            sprintf( name, "%s.node%05d.%d", imagedir.c_str(), index, i );
            child->name = name;
            child->parent = node;
            node->children[name] = child;
            child->camera = camera;
            camera->node = child;
            
            child->pose.so3() = catadioptric->Ry[i].inverse();
        }
    }
    
    void finish( Reconstruction &r )
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
    
    const Catadioptric *catadioptric;
    
    std::string imagedir;
    int index;
    
    Node *node;
    std::string prefix;
    
    bool delete_catadioptric;
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
        catadioptric = new Catadioptric( pathocam, calibration, sz, NFACES );
        std::cout << "done.\n";
    }
    
    std::vector<ReconstructionThread*> threads;
    for ( int i = begin_index; i <= end_index; i += step_index )
    {
        if ( threads.size() == NTHREADS ) {
            finishThreads( r, threads );
        }
        
        ExtractThread *mythread = NULL;
        
        if ( use_previous ) {
            char nodename[256];
            sprintf( nodename, "%s.node%05d", imagedir.c_str(), i );
            if ( r.nodes.count( nodename ) == 0 ) {
                fprintf( stderr, "could not find node %s\n", nodename );
                continue;
            }
            Node *node = (Node *)r.nodes[nodename];
            
            std::cout << "creating catadioptric unwarper for " << nodename << "...\n";
            Catadioptric *mycatadioptric = new Catadioptric( pathocam, calibration, sz, NFACES, node->pose.so3() );
            std::cout << "done.\n";
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
