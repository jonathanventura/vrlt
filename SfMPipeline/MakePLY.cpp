
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

using namespace vrlt;

struct PLYPoint
{
    Eigen::Vector3d X;
    Eigen::Vector3d N;
    unsigned char color[3];
    
    PLYPoint( const Eigen::Vector3d &_X, const Eigen::Vector3d &_N, const unsigned char *_color )
    {
        X = _X;
        N = _N;
        color[0] = _color[0];
        color[1] = _color[1];
        color[2] = _color[2];
    }
    
    void write( FILE *f )
    {
        fprintf( f, "%g %g %g\n", X[0], X[1], X[2] );
        fprintf( f, "%g %g %g\n", N[0], N[1], N[2] );
        fprintf( f, "%d %d %d\n", (int)color[0], (int)color[1], (int)color[2] );
    }
};

void writePLYHeader( FILE *f, size_t N )
{
    fprintf( f, "ply\n");
    fprintf( f, "format ascii 1.0\n" );
    fprintf( f, "element vertex %lu\n", N );
    fprintf( f, "property float x\n" );
    fprintf( f, "property float y\n" );
    fprintf( f, "property float z\n" );
    fprintf( f, "property float nx\n" );
    fprintf( f, "property float ny\n" );
    fprintf( f, "property float nz\n" );
    fprintf( f, "property uchar red\n" );
    fprintf( f, "property uchar green\n" );
    fprintf( f, "property uchar blue\n" );
    fprintf( f, "end_header\n" );
}

int main( int argc, char **argv )
{
    if ( argc != 1 &&  argc != 2 && argc != 3 ) {
        fprintf( stderr, "usage: %s <filename (without .xml)> [<group>]\n", argv[0] );
        exit(1);
    }
    
    std::string prefix = ".";
    std::string filename = "reconstruction";
    if ( argc > 1 ) filename = std::string( argv[1] );
    
    int group = -1;
    
    if ( argc == 3 ) {
        group = atoi(argv[2]);
    }
    
    std::stringstream pathin;
    pathin << prefix << "/" << filename << ".xml";
    std::stringstream pathout;
    if ( group == -1 ) {
        pathout << prefix << "/" << filename << ".ply";
    } else {
        pathout << prefix << "/" << filename << group << ".ply";
    }
    
    Reconstruction r;
    r.pathPrefix = prefix;
    XML::read( r, pathin.str() );
    
    Node *root = (Node*)r.nodes["root"];
    
    std::vector<PLYPoint> ply_points;
    std::vector<PLYPoint> ply_cameras;

    std::stringstream camerapath;
    camerapath << prefix << "/" << filename << ".cameras.dat";
    std::stringstream pointpath;
    pointpath << prefix << "/" << filename << ".points.dat";
    
    ElementList::iterator it;
    
    int ncameras = 0;
    for ( it = r.cameras.begin(); it != r.cameras.end(); it++ )
    {
        Camera *camera = (Camera *)it->second;
        if ( camera->node == NULL ) continue;
        if ( root != NULL && camera->node->root() != root ) continue;
        ncameras++;
    }
    
    if ( root != NULL ) {
        for ( it = root->points.begin(); it != root->points.end(); it++ )
        {
            Point *point = (Point *)it->second;
            Track *track = point->track;
            bool good = true;
            ElementList::iterator featit;
            for ( featit = track->features.begin(); featit != track->features.end(); featit++ )
            {
                Feature * feature = (Feature*)featit->second;
                Camera *camera = feature->camera;
                Node *node = camera->node;
                if ( node == NULL ) continue;
                if ( node->root() != root ) continue;
            }
            if ( !good ) continue;
                    
            Eigen::Vector3d X = project( point->position );
            
            Eigen::Vector3d N = point->normal;
            N.normalize();
            unsigned char *color = ((Feature*)(point->track->features.begin()->second))->color;
            
            ply_points.push_back( PLYPoint( X, N, color ) );
        }
    }
    
    for ( it = r.cameras.begin(); it != r.cameras.end(); it++ )
    {
        Camera *camera = (Camera *)it->second;
        if ( camera->node == NULL ) continue;
        if ( root != NULL && camera->node->root() != root ) continue;
        Sophus::SE3d pose = camera->node->globalPose();
        Eigen::Vector3d c = - (pose.so3().inverse() * pose.translation());
        
        Eigen::Vector3d N;
        N << 0, 0, 1;
        N = pose.so3().inverse() * N;
        
        unsigned char color[3] = { 0, 0, 255 };
        
        ply_cameras.push_back( PLYPoint( c, N, color ) );
    }
    
    FILE *f = fopen( pathout.str().c_str(), "w" );
    writePLYHeader( f, ply_points.size() + ply_cameras.size() );
    for ( int i = 0; i < ply_points.size(); i++ ) ply_points[i].write( f );
    for ( int i = 0; i < ply_cameras.size(); i++ ) ply_cameras[i].write( f );
    fclose( f );
    
    std::stringstream cameraplypath;
    cameraplypath << prefix << "/" << filename << ".cameras.ply";
    f = fopen( cameraplypath.str().c_str(), "w" );
    writePLYHeader( f, ply_cameras.size() );
    for ( int i = 0; i < ply_cameras.size(); i++ ) ply_cameras[i].write( f );
    fclose( f );
    
    return 0;
}
