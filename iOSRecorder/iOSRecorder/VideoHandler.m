/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: VideoHandler.m
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import "VideoHandler.h"
#import "MainViewController.h"

#include <MobileCoreServices/MobileCoreServices.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <unistd.h>

uint64_t ConvertMachToNanoseconds( uint64_t t );

static mach_timebase_info_data_t sTimebaseInfo;
uint64_t ConvertMachToNanoseconds( uint64_t t )
{
    return t * sTimebaseInfo.numer / sTimebaseInfo.denom;
}

static uint64_t last_frame_time = 0;
static double total_elapsed = 0;
static size_t ntimings = 0;

@implementation VideoHandler
@synthesize controller;
@synthesize frameRate;

- (id)init
{
    if ( ( self = [super init] ) )
    {
        (void) mach_timebase_info(&sTimebaseInfo);

        frameRate = 30;
        
//        motionManager = [[CMMotionManager alloc] init];
//        motionManager.gyroUpdateInterval = 0.01;
        
//        locationManager = [[CLLocationManager alloc] init];
//        locationManager.headingOrientation = CLDeviceOrientationUnknown;
//        locationManager.headingFilter = kCLHeadingFilterNone;
        
//        [motionManager startDeviceMotionUpdates];
//        [locationManager startUpdatingLocation];
//        [locationManager startUpdatingHeading];
    }
    
    return self;
}

- (void)dealloc
{
//    [motionManager release];
    
//    [locationManager release];
    
    [super dealloc];
}

- (void)setDeviceOrientation:(UIInterfaceOrientation)orientation
{
    switch ( orientation ) {
        case UIInterfaceOrientationPortrait:
            locationManager.headingOrientation = CLDeviceOrientationPortrait;
            break;
            
        case UIInterfaceOrientationLandscapeLeft:
            locationManager.headingOrientation = CLDeviceOrientationLandscapeLeft;
            break;

        case UIInterfaceOrientationLandscapeRight:
            locationManager.headingOrientation = CLDeviceOrientationLandscapeRight;
            break;

        case UIInterfaceOrientationPortraitUpsideDown:
            locationManager.headingOrientation = CLDeviceOrientationPortraitUpsideDown;
            break;
    }
}

- (void)startRecordingWithTitle:(NSString *)title
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];

    //imagesDirectory = [[NSDate date] description];
    imagesDirectory = title;
    [imagesDirectory retain];
    
    imagesPath = [NSString stringWithFormat:@"%@/%@",documentsDirectory,imagesDirectory];
    [imagesPath retain];

    xmlPath = [imagesPath stringByAppendingString:@".xml"];
    [xmlPath retain];
    
    //NSLog( @"images path: %@", imagesPath );
    //NSLog( @"xml path: %@", xmlPath );
    
    /*
    BOOL success = [[NSFileManager defaultManager] createDirectoryAtPath:imagesPath withIntermediateDirectories:NO attributes:nil error:nil];
    
    if ( !success )
    {
        NSLog( @"could not create images directory" );
        [imagesPath release];
        [xmlPath release];
        return;
    }
    */
    
    xmlFile = fopen( [xmlPath UTF8String], "w" );
    
    fprintf( xmlFile, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" );
    fprintf( xmlFile, "<reconstruction>\n" );
    
    //NSString *model = [[UIDevice currentDevice] model];
    //NSLog( @"model: %@", model );
    
    fprintf( xmlFile, "<calibration name=\"ipad\">1179.90411 639.50000 359.50000 0 0</calibration>\n" );
    
    frameno = 1;
    
    [startDate release];
    startDate = [[NSDate date] retain];
    
    
    NSString *videopath = [imagesPath stringByAppendingString:@".mp4"];
    [videopath retain];
    
    NSError *err;
    assetWriter = [[AVAssetWriter alloc] initWithURL:[NSURL fileURLWithPath:videopath] fileType:AVFileTypeMPEG4 error:&err];
    
    NSDictionary *outputSettings = [NSDictionary dictionaryWithObjectsAndKeys:
                                    AVVideoCodecH264,AVVideoCodecKey,
                                    [NSNumber numberWithInt:1280],AVVideoWidthKey,
                                    [NSNumber numberWithInt:720],AVVideoHeightKey,
                                    nil];
    assetWriterInput = [[AVAssetWriterInput alloc] initWithMediaType:AVMediaTypeVideo outputSettings:outputSettings];
    assetWriterInput.expectsMediaDataInRealTime = YES;
    
    NSDictionary *pixbufAttr = [NSDictionary dictionaryWithObjectsAndKeys:
                                [NSNumber numberWithInt:kCVPixelFormatType_32BGRA],kCVPixelBufferPixelFormatTypeKey,
                                [NSNumber numberWithInt:1280],kCVPixelBufferWidthKey,
                                [NSNumber numberWithInt:720],kCVPixelBufferHeightKey,
                                nil];
    assetWriterInputAdaptor = [[AVAssetWriterInputPixelBufferAdaptor alloc] initWithAssetWriterInput:assetWriterInput sourcePixelBufferAttributes:pixbufAttr];
    
    
    [assetWriter addInput:assetWriterInput];
    
    [assetWriter startWriting];
    
    assetNextTime = CMTimeMake(0,frameRate);
    [assetWriter startSessionAtSourceTime:assetNextTime];

    
    
    
    isRecording = TRUE;
}

- (void)endRecording
{
    if ( !isRecording ) return;
    
    fprintf( xmlFile, "</reconstruction>\n" );
    
    fclose( xmlFile );
    
    [imagesDirectory release];
    [imagesPath release];
    [xmlPath release];
    
    [assetWriter finishWriting];
    
    /*
    BOOL isCompatible = UIVideoAtPathIsCompatibleWithSavedPhotosAlbum( [assetWriter.outputURL path] );
    if ( isCompatible ) {
        UISaveVideoAtPathToSavedPhotosAlbum( [assetWriter.outputURL path], nil, nil, nil );
    }
    */
    
    [assetWriter release];
    [assetWriterInput release];
    [assetWriterInputAdaptor release];
    
    assetWriter = nil;

    
    isRecording = FALSE;
}

- (void)processMotion
{
    timestamp = [[NSDate date] timeIntervalSinceDate:startDate];
    
    CMDeviceMotion *deviceMotion = [motionManager.deviceMotion retain];
    
    rotationRate[0] = deviceMotion.rotationRate.x;
    rotationRate[1] = deviceMotion.rotationRate.y;
    rotationRate[2] = deviceMotion.rotationRate.z;
    
    gravity[0] = deviceMotion.gravity.x;
    gravity[1] = deviceMotion.gravity.y;
    gravity[2] = deviceMotion.gravity.z;
    
    [deviceMotion release];
    
    CLLocation *location = [locationManager.location retain];
    
    altitude = location.altitude;
    verticalAccuracy = location.verticalAccuracy;
    
    latitude = location.coordinate.latitude;
    longitude = location.coordinate.longitude;
    horizontalAccuracy = location.horizontalAccuracy;
    
    [location release];
    
    CLHeading *headingObj = [locationManager.heading retain];
    
    heading = headingObj.trueHeading;
    headingAccuracy = headingObj.headingAccuracy;
    
    [headingObj release];
    
    //NSLog( @"location timestamp: %@", [location.timestamp description] );
    
}

- (void)processFrame
{
    [self processMotion];
    
    /*
    // check that location info is good
    if ( gravity[0] == 0 || altitude == 0 ) return;
    */
    
    /*
    NSString *jpegPath = [imagesPath stringByAppendingFormat:@"/frame%06d.jpg",frameno];
    [UIImageJPEGRepresentation( image, 0.0 ) writeToFile:jpegPath atomically:YES];
    */
    
    fprintf( xmlFile, "\t<image name=\"frame%06d\" calibration=\"ipad\" ", frameno );
    fprintf( xmlFile, "timestamp=\"%.15lf\" ", timestamp );
    fprintf( xmlFile, "up=\"%.15lf %.15lf %.15lf\" ", -gravity[0], -gravity[1], -gravity[2] );
    fprintf( xmlFile, "latitude=\"%.15lf\" longitude=\"%.15lf\" ", latitude, longitude );
    fprintf( xmlFile, "gpsRadius=\"%lf\" ", horizontalAccuracy );
    fprintf( xmlFile, "altitude=\"%.15lf\" ", altitude );
    fprintf( xmlFile, "altitudeRadius=\"%lf\" ", verticalAccuracy );
    fprintf( xmlFile, "heading=\"%.15lf\" ", heading );
    fprintf( xmlFile, "headingAccuracy=\"%lf\" ", headingAccuracy );
    fprintf( xmlFile, ">%s/frame%06d.jpg</image>\n", [imagesDirectory UTF8String], frameno );
    
    frameno++;
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    uint64_t this_frame_time = mach_absolute_time();
    if ( last_frame_time != 0 )
    {
        uint64_t elapsed_mach = this_frame_time - last_frame_time;
        uint64_t elapsed_nano = ConvertMachToNanoseconds(elapsed_mach);
        double elapsed = elapsed_nano * 1e-9;
        total_elapsed += elapsed;
        ntimings++;
        
        if ( ntimings == 1000 )
        {
            double avg = total_elapsed / ntimings;
            NSLog( @"average frame time: %g s (%g fps)", avg, 1./avg );
            total_elapsed = 0.;
            ntimings = 0;
        }
    }
    last_frame_time = this_frame_time;
    
    
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

    if ( isRecording ) {
        if ( assetWriterInput.isReadyForMoreMediaData )
        {
            [assetWriterInputAdaptor appendPixelBuffer:imageBuffer withPresentationTime:assetNextTime];
            assetNextTime.value++;
            
//            [self processFrame];
        }
        else
        {
            NSLog( @"not ready" );
        }
    }
}

@end
