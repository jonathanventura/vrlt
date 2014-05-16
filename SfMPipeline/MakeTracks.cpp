
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <iostream>
#include <set>
#include <deque>

using namespace vrlt;

class TrackMaker
{
public:
    TrackMaker( int _min_track_size = 3 ) : min_track_size( _min_track_size ), num_inconsistent( 0 ), num_too_small( 0 ), count( 0 ) { }
    
    Track* make( Feature *feature_in )
    {
        if ( visited( feature_in ) ) return NULL;
        
        Track *track = new Track;
        
        // breadth-first search for neighbors
        visited_cameras.clear();
        visited_nodes.clear();
        
        std::deque<Feature*> features;
        features.push_back( feature_in );
        while ( !features.empty() )
        {
            Feature *feature = features.front();
            features.pop_front();
            
            if ( visited( feature ) ) continue;
            
            visited_features.insert( feature );
            
            track->features[feature->name] = feature;
            
            
            ElementList::iterator matchit;
            for ( matchit = feature->matches.begin(); matchit != feature->matches.end(); matchit++ )
            {
                Match *match = (Match*) matchit->second;
                Feature *matched_feature = NULL;
                
                if ( match->feature1->name == feature->name ) {
                    matched_feature = match->feature2;
                } else {
                    matched_feature = match->feature1;
                }
                
                features.push_back( matched_feature );
            }
        }

        // consistency check -- make sure no image is in track twice
        bool consistent = checkTrackConsistency( track );
        
        if ( !consistent )
        {
            num_inconsistent++;
            delete track;
            return NULL;
        }
        
        if ( track->features.size() < min_track_size )
        {
            num_too_small++;
            delete track;
            return NULL;
        }
    
        char name[256];
        sprintf( name, "track%lu", count++ );
        track->name = std::string(name);
        
        return track;
    }
    
    size_t num_inconsistent;
    size_t num_too_small;
protected:
    int min_track_size;
    std::set<Feature*> visited_features;
    std::set<Node*> visited_nodes;
    std::set<Camera*> visited_cameras;
    size_t count;
    
    bool visited( Feature *feature )
    {
        return ( visited_features.count( feature ) > 0 );
    }
    
    bool visited( Camera *camera )
    {
        return ( visited_cameras.count( camera ) > 0 );
    }
    
    bool visited( Node *node )
    {
        return ( visited_nodes.count( node ) > 0 );
    }
    
    bool checkTrackConsistency( Track *track )
    {
        std::set<Camera*> cameras;
        std::vector<Feature*> features_to_remove;
        
        ElementList::iterator it;
        for ( it = track->features.begin(); it != track->features.end(); it++ )
        {
            Feature *feature = (Feature*) it->second;
            Camera *camera = feature->camera;
            
            if ( cameras.count( camera ) > 0 ) return false;
            cameras.insert( camera );
        }
        
        return true;

    }

};

int main( int argc, char **argv )
{
    if ( argc != 3 && argc != 4 ) {
        fprintf( stderr, "usage: %s <file in> <file out> [<min track size>]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    std::string pathout = std::string(argv[2]);
    int min_track_size = 3;
    if ( argc == 4 ) min_track_size = atoi(argv[3]);
    
    Reconstruction r;
    XML::read( r, pathin );
    
    TrackMaker trackMaker( min_track_size );
    
    ElementList::iterator it;
    for ( it = r.features.begin(); it != r.features.end(); it++ )
    {
        Feature *feature = (Feature*) it->second;
        
        Track *track = trackMaker.make( feature );
        
        if ( track != NULL ){
            r.tracks[track->name] = track;
        }
    }
    
    std::cout << "made " << r.tracks.size() << " tracks\n";
    std::cout << trackMaker.num_inconsistent << " inconsistent\n";
    std::cout << trackMaker.num_too_small << " too small\n";
    
    XML::write( r, pathout );
}
