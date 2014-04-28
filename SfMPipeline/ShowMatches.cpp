
#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <opencv2/highgui.hpp>

#include <iostream>

using namespace vrlt;

void showMatches( Reconstruction &r, Camera *camera1, Camera *camera2 )
{
    cv::Mat image1 = cv::imread( camera1->path, cv::IMREAD_COLOR );
    cv::Mat image2 = cv::imread( camera2->path, cv::IMREAD_COLOR );
    
    cv::Mat imageout( cv::Size( image1.size().width + image2.size().width, image1.size().height ), CV_8UC3 );
    
    image1.copyTo( imageout( cv::Rect( cv::Point2i(0,0), image1.size() ) ) );
    image2.copyTo( imageout( cv::Rect( cv::Point2i(image1.size().width,0), image2.size() ) ) );
    int offsetx = image1.size().width;
    
    int count = 0;
    
    ElementList::iterator it;
    for ( it = r.matches.begin(); it != r.matches.end(); it++ )
    {
        Match *match = (Match*)it->second;
        
        Feature *feature1 = NULL;
        Feature *feature2 = NULL;
        
        if ( camera1 == match->feature1->camera ) {
            feature1 = match->feature1;
        } else if ( camera1 == match->feature2->camera ) {
            feature1 = match->feature2;
        }
        
        if ( camera2 == match->feature1->camera ) {
            feature2 = match->feature1;
        } else if ( camera2 == match->feature2->camera ) {
            feature2 = match->feature2;
        }
        
        if ( !feature1 || !feature2 ) continue;
        count++;
        
        cv::Point2i pt1( feature1->location[0], feature1->location[1] );
        cv::Point2i pt2( offsetx + feature2->location[0], feature2->location[1] );
        
        cv::line( imageout, pt1, pt2, cv::Scalar( 255, 255, 255 ) );
        
        cv::circle( imageout, pt1, feature1->scale, cv::Scalar(0,0,0) );
        cv::circle( imageout, pt2, feature2->scale, cv::Scalar(0,0,0) );
    }
    
    if ( count == 0 ) return;
    
    char path[256];
    sprintf( path, "Output/%s.%s.jpg", camera1->name.c_str(), camera2->name.c_str() );
    std::cout << path << "\n";
    cv::imwrite( path, imageout );
}

int main( int argc, char **argv )
{
    if ( argc != 2 && argc != 4 ) {
        fprintf( stderr, "usage: %s <file in> [<image1> <image2>]\n", argv[0] );
        exit(1);
    }
    
    std::string pathin = std::string(argv[1]);
    
    Reconstruction r;
    XML::read( r, pathin );
    
    if ( argc == 4 ) {
        std::string name1 = std::string(argv[2]);
        std::string name2 = std::string(argv[3]);

        if ( r.cameras.count(name1) == 0 ) {
            std::cout << "could not find " << name1 << "\n";
            exit(1);
        }
        
        if ( r.cameras.count(name2) == 0 ) {
            std::cout << "could not find " << name2 << "\n";
            exit(1);
        }
        
        Camera *camera1 = (Camera*) r.cameras[name1];
        Camera *camera2 = (Camera*) r.cameras[name2];
        
        showMatches( r, camera1, camera2 );
    } else {
        ElementList::iterator it1;
        for ( it1 = r.cameras.begin(); it1 != r.cameras.end(); it1++ )
        {
            Camera *camera1 = (Camera*)it1->second;
            
            ElementList::iterator it2 = it1;
            for ( it2++; it2 != r.cameras.end(); it2++ )
            {
                Camera *camera2 = (Camera*)it2->second;
                
                if ( camera1->node->pose.so3().log()[1] != camera2->node->pose.so3().log()[1] ) continue;
                
                showMatches( r, camera1, camera2 );
            }
        }
    }
}
