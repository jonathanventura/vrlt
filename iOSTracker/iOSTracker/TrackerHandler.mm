
#include <MultiView/multiview_io_xml.h>

#include <PatchTracker/search.h>

#import "TrackerHandler.h"
#import "VideoHandler.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace vrlt;

@implementation TrackerHandler
@synthesize maxnumpoints;
@synthesize needLocalize;
@synthesize videoHandler;
@synthesize calibration;
@synthesize root;
@synthesize node;
@synthesize doTrack;
@synthesize tracked;

//#define PRELOAD_CACHE
//#define USE_LOCALIZER

//#define SEARCH_CACHE
//#define ADD_TO_CACHE
#define ADD_CAMERA

static void copyImageData( UIImage *image, Camera *camera )
{
    // from http://stackoverflow.com/questions/448125/how-to-get-pixel-data-from-a-uiimage-cocoa-touch-or-cgimage-core-graphics
    
    CGImageRef imageRef = [image CGImage];
    NSUInteger width = CGImageGetWidth(imageRef);
    NSUInteger height = CGImageGetHeight(imageRef);
    camera->image.create( cv::Size( width, height ), CV_8UC1 );
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceGray();
    unsigned char *rawData = camera->image.data;
    NSUInteger bytesPerPixel = 1;
    NSUInteger bytesPerRow = bytesPerPixel * width;
    NSUInteger bitsPerComponent = 8;
    CGContextRef context = CGBitmapContextCreate(rawData, width, height,
                                                 bitsPerComponent, bytesPerRow, colorSpace,
                                                 kCGImageAlphaNone | kCGBitmapByteOrderDefault);
    CGColorSpaceRelease(colorSpace);
    
    CGContextDrawImage(context, CGRectMake(0, 0, width, height), imageRef);
    CGContextRelease(context);
}

- (id)initWithPrefix:(NSString *)prefix scale:(int)theScale shift:(int)theShift
{
    if ( ( self = [super init] ) )
    {
        NSLog( @"loading..." );
        
        fpsTimerTotal = 0.;
        
        imsize = cv::Size( 1280, 720 );
        
        calibration = new Calibration;
        calibration->name = "ipad";
        // iPhone
        //calibration->focal = 1489.653430;
        // iPad
        calibration->focal = 1179.90411;
        calibration->center[0] = 639.500000;
        calibration->center[1] = 359.500000;
        
        camera = new Camera;
        camera->name = "camera";
        camera->calibration = calibration;
        camera->image.create( imsize, CV_8UC1 );
        camera->pyramid.resize( imsize );
        node = new Node;
        node->name = "node";
        node->camera = camera;
        camera->node = node;
        
        newcamera = NULL;
        
        imagecache = new ImageCache();
        imagecachelock = [[NSLock alloc] init];

        r = new Reconstruction;
        r->pathPrefix = string( [prefix UTF8String] );
        
        stringstream path;
        path << r->pathPrefix << "/reconstruction.xml";
        //path << r->pathPrefix << "/pmvs.xml";
        XML::read( *r, string( path.str() ) );
        
        root = (Node *)r->nodes["root"];
        
        ElementList::iterator it;
        for ( it = r->cameras.begin(); it != r->cameras.end(); it++ )
        {
            Camera *mycamera = (Camera *)it->second;
            UIImage *myimage = [[UIImage alloc] initWithContentsOfFile:[NSString stringWithFormat:@"%@/%s",prefix,mycamera->path.c_str()]];
            copyImageData( myimage, mycamera );
            [myimage release];
            mycamera->pyramid = ImagePyramid( mycamera );
        }
        
        maxnumpoints = 1024;
        minnumpoints = 100;
        minratio = .1;
        
        ntimeslost = 100;
        
#ifdef PRELOAD_CACHE
        for ( it = r->cameras.begin(); it != r->cameras.end(); it++ )
        {
            Camera *mycamera = (Camera *)it->second;
            imagecache->makeSmallByteImage( mycamera );
            imagecache->prepareSmallImage( mycamera );
            imagecache->add( mycamera );
        }
#endif
        
        tracker = new Tracker(root,maxnumpoints);
        tracker->verbose = false;
        tracker->niter = 10;
        tracker->firstlevel = 3;
        tracker->lastlevel = 0;
        doTrack = YES;
        
        tracker->minnumpoints = minnumpoints;
        tracker->minratio = minratio;
        
        needLocalize = NO;
        localizer = NULL;
        localizerlock = [[NSLock alloc] init];
        
        localizerResponses = [[NSMutableArray alloc] init];
        localizerResponsesLock = [[NSLock alloc] init];
        
        nFpsTimerFrames = 0;
        
        localizer = NULL;
        
#ifdef USE_LOCALIZER
        scale = theScale;
        shift = theShift;
        localizer = new LocalizationClient( scale, shift );
        bool connected;
        for ( int i = 0; i < 10; i++ ) {
            connected = localizer->connectToServer( "192.168.0.106", 12345+i );
            if ( connected ) break;
        }
        if ( !connected ) {
            NSLog( @"could not connect" );
            exit(0);
        }
#endif
        
        data2 = (uchar*)malloc( 640*360 );
        data4 = (uchar*)malloc( 320*180 );
        data8 = (uchar*)malloc( 160*90 );
        packedData = (uchar*)malloc( 1280*720 );
        
        NSLog( @"client ready." );
        
        robustlsq = new RobustLeastSq( root );
    }
    
    return self;
}

- (void)dealloc
{
    delete imagecache;
    delete tracker;

    delete calibration;
    delete camera;
    delete node;
    
    delete r;
    
    delete localizer;

    [imagecachelock release];
    [localizerlock release];
    
    free( data2 );
    free( data4 );
    free( data8 );
    
    [super dealloc];
}

- (void)setGrayData:(unsigned char *)grayData
{
    cv::Mat im( imsize, CV_8UC1, grayData );
    im.copyTo( camera->image );
    camera->pyramid.levels[0].image = camera->image;
    camera->pyramid.remake();
}

- (int)numFramesInCache
{
    return imagecache->cache.size();
    return 0;
}

- (void)process
{
    NSDate *fpsTimer = [[NSDate date] retain];
    
    if ( ntimeslost > 30 )
    {
#ifdef SEARCH_CACHE
        [imagecachelock lock];
        imagecache->search( camera );
        [imagecachelock unlock];
#endif
    }
    
    // check for new localization responses
    // we do this here on the main thread because of the CVD allocations and frees
    [localizerResponsesLock lock];
    for ( LocalizationRequest *response in localizerResponses ) {
        Sophus::SE3d pose = Sophus::SE3d::exp( Eigen::Map< Eigen::Matrix<double,6,1> >( response.posedata ) );
        
        if ( ntimeslost > 30 ) {
            camera->node->pose.so3() = (videoHandler.currentAttitude * response.attitude.inverse()) * pose.so3();
            camera->node->pose.translation() = (videoHandler.currentAttitude * response.attitude.inverse()) * pose.translation();
        }
        
        [response release];
        
#ifdef ADD_TO_CACHE
        [imagecachelock lock];
        if ( imagecache->test( camera ) )
        {
            NSLog( @"ADDING TO CACHE" );
            imagecache->addCopy( camera );
        }
        [imagecachelock unlock];
#endif
    }
    [localizerResponses removeAllObjects];
    [localizerResponsesLock unlock];
    
    tracked = tracker->track( camera );
    if ( tracked ) robustlsq->run( camera );
    
    NSLog( @"tracked %d/%d points", tracker->ntracked, tracker->nattempted );
    
    if ( tracked ) {
#ifdef ADD_CAMERA
        float newratio = (float)tracker->nnew / (float)tracker->ntracked;
        
        bool should_add = true;
        
        if ( newratio >= .5 ) {
            should_add = false;
            
            Sophus::SE3d trackerpose = camera->node->pose;
            Eigen::Vector3d trackercenter = -( trackerpose.so3().inverse() * trackerpose.translation() );
            
            double min_dist = INFINITY;
            
            // check distance to other new cameras
            for ( int i = 0; i < tracker->cameras.size(); i++ )
            {
                Camera *othercamera = tracker->cameras[i];
                if ( othercamera->isnew == false ) continue;
                
                Sophus::SE3d pose = othercamera->node->pose;
                Eigen::Vector3d center = -( pose.so3().inverse() * pose.translation() );
                
                double dist = ( center - trackercenter ).norm();
                if ( dist < min_dist ) min_dist = dist;
            }
            
            if ( min_dist > .1 ) should_add = true;
        }

        if ( should_add ) {
            if ( newcamera != NULL ) {
                removeCameraFeatures( *r, newcamera );
                root->children.erase( newcamera->node->name );
                r->cameras.erase( newcamera->name );
                delete newcamera;
                tracker->cameras.pop_back();
            }
            
            newcamera = addCameraToReconstruction( *r, camera->calibration, camera->image, camera->node->pose );
            tracker->cameras.push_back( newcamera );
            
            NSLog( @"Camera added!" );
        }
#endif        
        
        ntimeslost = 0;
        self.needLocalize = NO;
        
#ifdef ADD_TO_CACHE
        [imagecachelock lock];
        if ( imagecache->test( camera ) )
        {
            NSLog( @"ADDING TO CACHE" );
            imagecache->addCopy( camera );
        }
        [imagecachelock unlock];
#endif
    } else {
        ntimeslost++;
        
        if ( ntimeslost >= 30 ) {
            self.needLocalize = YES;
        }
    }
    
    fpsTimerTotal += -[fpsTimer timeIntervalSinceNow];
    [fpsTimer release];
    
    nFpsTimerFrames++;
    if ( nFpsTimerFrames == 100 )
    {
        double fps = 100. / fpsTimerTotal;
        [videoHandler reportFPS:fps];
        nFpsTimerFrames = 0;
        fpsTimerTotal = 0;
        
#ifndef USE_DISPATCH
        NSLog( @"average cull time: %0.5f ms", tracker->cullTimer.average()*1e-6 );
        NSLog( @"average warp time: %0.5f ms", tracker->patchSearcher->warpTimer.average()*1e-6 );
        NSLog( @"average search time: %0.5f ms", tracker->patchSearcher->searchTimer.average()*1e-6 );
#endif
    }
}

static NSData * getJPEGData( const cv::Mat &image, float quality )
{
    size_t width = image.size().width;
    size_t height = image.size().height;
    
    CGDataProviderRef dataProviderRef = CGDataProviderCreateWithData( NULL, image.data, width*height, NULL );
    
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceGray();
    size_t bitsPerPixel = 8;
    size_t bytesPerRow = width;
    size_t bitsPerComponent = 8;
    CGImageRef imageRef = CGImageCreate( width, height,
                  bitsPerComponent, bitsPerPixel, bytesPerRow, colorSpace, kCGImageAlphaNone,
                  dataProviderRef, NULL, false, kCGRenderingIntentDefault );
    
    UIImage *uiimage = [[UIImage imageWithCGImage:imageRef] retain];

    NSData *jpegData = [UIImageJPEGRepresentation( uiimage, quality ) retain];
    
    [uiimage release];
    CGDataProviderRelease( dataProviderRef );
    CGImageRelease( imageRef );
    CGColorSpaceRelease(colorSpace);
    
    return jpegData;
}

- (void)requestLocalization:(id)arg
{
    LocalizationRequest *request = (LocalizationRequest*)arg;
    
    if ( localizer == NULL )
    {
        [request release];
        return;
    }
    
    if ( [localizerlock tryLock] == FALSE )
    {
        [request release];
        return;
    }
    
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    
    int w = 1280;
    int h = 720;
    unsigned char *data = request.imagedata;
    
    NSData *jpegData = getJPEGData( cv::Mat( cv::Size( w, h ), CV_8UC1, data ), 0.2 );
    localizer->sendImage( jpegData.length, (unsigned char *) jpegData.bytes );
    [jpegData release];
    
    BOOL success = FALSE;
    for ( int i = 0; i < 6; i++ ) if ( request.posedata[i] != 0 ) success = TRUE;
    if ( success )
    {
        [localizerResponsesLock lock];
        [localizerResponses addObject:request];
        [localizerResponsesLock unlock];
    } else {
        [request release];
    }
    
    [localizerlock unlock];
    
    [pool release];
}

@end


@implementation LocalizationRequest
@synthesize attitude;
@synthesize imagedata;
@synthesize posedata;

- (id)initWithImageData:(const unsigned char *)data attitude:(Sophus::SO3d)att
{
    if ( ( self = [super init] ) )
    {
#ifdef USE_LOCALIZER
        imagedata = (unsigned char *)malloc( 1280*720 );
        posedata = (double *)malloc( 6*sizeof(double) );
        memcpy( imagedata, data, 1280*720 );
        attitude = att;
#endif
    }
    
    return self;
}

- (void)dealloc
{
#ifdef USE_LOCALIZER
    free( imagedata );
    free( posedata );
#endif
    [super dealloc];
}

@end