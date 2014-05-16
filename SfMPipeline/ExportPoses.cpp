
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <sys/stat.h>

using namespace vrlt;

static void writeOutGluLookAt( Reconstruction &r, const std::string &path )
{
    FILE *f = fopen( path.c_str(), "w" );

    fprintf( f, "focal width height\n" );
    Calibration *calibration = (Calibration*)r.calibrations.begin()->second;
    
    Camera *camera = (Camera*)r.cameras.begin()->second;
    cv::Mat image = cv::imread( camera->path );
    
    fprintf( f, "%lf %d %d\n", calibration->focal, image.size().width, image.size().height );

    // my coordinate system: -Y is up
    // UTM coordinate system: +Z is up
    Eigen::Matrix3d convertToUTM;
    convertToUTM <<
    1, 0, 0,
    0, 0, 1,
    0, -1, 0;
    
    fprintf( f, "filename eyeX eyeY eyeZ centerX centerY centerZ upX upY upZ\n" );
    
    for ( ElementList::iterator it = r.cameras.begin(); it != r.cameras.end(); it++ )
    {
        Camera *camera = (Camera*)it->second;
        
        Node *node = camera->node;
        if ( node == NULL ) continue;
        if ( node->parent == NULL ) continue;
        
        Eigen::Vector3d eye;
        Eigen::Vector3d center;
        Eigen::Vector3d up;
        
        getGluLookAtVectors( node->globalPose(), eye, center, up );
        
        if ( r.utmZone != 0 )
        {
            eye = convertToUTM * eye;
            center = convertToUTM * center;
            up = convertToUTM * up;
            
            eye[0] += r.utmCenterEast;
            eye[1] += r.utmCenterNorth;
        }
        
        fprintf( f, "%s ", camera->path.c_str() );
        fprintf( f, "%0.17lf %0.17lf %0.17lf ", eye[0], eye[1], eye[2] );
        fprintf( f, "%0.17lf %0.17lf %0.17lf ", center[0], center[1], center[2] );
        fprintf( f, "%0.17lf %0.17lf %0.17lf\n", up[0], up[1], up[2] );
    }
    
    fclose( f );
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

    writeOutGluLookAt( r, pathout );
}
