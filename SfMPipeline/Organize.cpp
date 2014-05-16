
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <sys/stat.h>

using namespace vrlt;

static void writeOutGluLookAt( Reconstruction &r, const char *path )
{
    FILE *f = fopen( path, "w" );
    
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
    if ( argc != 3 && argc != 4 ) {
        fprintf( stderr, "usage: %s <file in> <directory out> [<min track size>]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string dirout = std::string(argv[2]);
    int min_track_size = 0;
    if ( argc == 4 ) min_track_size = atoi( argv[3] );
    
    Reconstruction r;
    XML::read( r, pathin );
    
    mkdir( dirout.c_str(), 0777 );
    chmod( dirout.c_str(), 07777 );
    int count = 0;
    
    Node *root = (Node*)r.nodes["root"];
    
    r.matches.clear();
    r.pairs.clear();
    
    std::vector<Camera*> cameras_to_remove;
    
    ElementList::iterator it;
    for ( it = r.cameras.begin(); it != r.cameras.end(); it++ )
    {
        Camera *camera = (Camera *)it->second;
        
        if ( camera->node == NULL ) {
            cameras_to_remove.push_back( camera );
            continue;
        }
        
        if ( camera->node->root() != root ) {
            cameras_to_remove.push_back( camera );
            continue;
        }
        
        cv::Mat colorim = cv::imread( camera->path, cv::IMREAD_COLOR );
        
        std::stringstream newfilename;
        newfilename << count++ << ".jpg";
        
        std::stringstream newpath;
        newpath << dirout << "/" << newfilename.str();

        r.pathPrefix = ".";
        XML::readDescriptors( r, camera );
        camera->path = newfilename.str();
        r.pathPrefix = dirout.c_str();
        XML::writeFeatures( r, camera );
        XML::writeDescriptors( r, camera );
        
        cv::imwrite( newpath.str(), colorim );
    }
    
    for ( int i = 0; i < cameras_to_remove.size(); i++ ) {
        Camera *camera = (Camera*)r.cameras[ cameras_to_remove[i]->name ];
        for ( it = camera->features.begin(); it != camera->features.end(); it++ )
        {
            Feature *feature = (Feature*)it->second;
            Track *track = feature->track;
            if ( track == NULL ) continue;
            track->features.erase( feature->name );
        }
        r.cameras.erase( cameras_to_remove[i]->name );
    }
    
    std::vector<Track*> tracks_to_remove;
    std::vector<Point*> points_to_remove;
    for ( it = r.tracks.begin(); it != r.tracks.end(); it++ )
    {
        Track *track = (Track*)it->second;
        if ( track->features.empty() ) tracks_to_remove.push_back( track );
        
        if ( track->point == NULL ) continue;
        
        if ( track->features.size() < min_track_size ) {
            tracks_to_remove.push_back( track );
            points_to_remove.push_back( track->point );
        }
        
    }
    for ( int i = 0; i < tracks_to_remove.size(); i++ ) r.tracks.erase( tracks_to_remove[i]->name );
    for ( int i = 0; i < points_to_remove.size(); i++ ) root->points.erase( points_to_remove[i]->name );
    
    r.nodes.clear();
    r.nodes["root"] = root;

    std::stringstream gluLookAtPath;
    gluLookAtPath << dirout << "/gluLookAt.txt";
    writeOutGluLookAt( r, gluLookAtPath.str().c_str() );
    
    std::stringstream pathout;
    pathout << dirout << "/reconstruction.xml";
    
    XML::write( r, pathout.str() );
}
