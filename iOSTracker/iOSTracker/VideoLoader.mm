//
//  VideoLoader.m
//  iOSTracker
//
//  Created by Jonathan Ventura on 7/16/14.
//
//

#import "VideoLoader.h"

#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <GLUtils/LoadImage.h>

using namespace vrlt;

@interface VideoLoader ()
{
    NSTimer *imagetimer;
    NSString *documentsDirectory;
    Reconstruction r;
    ElementList::iterator imageit;
}
- (void)loadNextImage:(id)sender;
@end


@implementation VideoLoader

- (id)initFromFile:(NSString*)path
{
    if ( ( self = [super init] ) )
    {
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        documentsDirectory = [[paths objectAtIndex:0] retain];
        
        NSString *xmlpath = [[NSString alloc] initWithFormat:@"%@/%@",documentsDirectory,path];

        XML::read( r, [xmlpath UTF8String] );
        /*
        for ( ElementList::iterator it = r.cameras.begin(); it != r.cameras.end(); it++ )
        {
            Camera *camera = (Camera*)it->second;
            
            std::stringstream imagepath;
            imagepath << [documentsDirectory UTF8String];
            imagepath << "/" << camera->path;
            
            loadJPEG_gray( imagepath.str().c_str(), camera->image );
        }
        */
    }
    
    return self;
}

- (void)start
{
    if ( imagetimer ) return;
    imageit = r.cameras.begin();
    imagetimer = [[NSTimer scheduledTimerWithTimeInterval:1./30. target:self selector:@selector(loadNextImage:) userInfo:nil repeats:YES] retain];
}

- (void)loadNextImage:(id)sender
{
    if ( imageit == r.cameras.end() ) {
        [imagetimer release];
        return;
    }
    
    Camera *camera = (Camera*)imageit->second;
    
    std::stringstream imagepath;
    imagepath << [documentsDirectory UTF8String];
    imagepath << "/" << camera->path;
    
//    NSLog( @"loading %s", imagepath.str().c_str() );
    loadJPEG_gray( imagepath.str().c_str(), camera->image );

    if ( camera->node->pose.translation().norm() == 0 )
    {
        [self.delegate processFrame:camera->image.data];
    }
    else
    {
        [self.delegate processFrame:camera->image.data withPose:camera->node->pose];
    }
    
    camera->image.create( cv::Size(0,0), CV_8UC1 );
    
    imageit++;
}

@end
