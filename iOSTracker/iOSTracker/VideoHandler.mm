/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: VideoHandler.mm
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import "VideoHandler.h"
#import "MainViewController.h"

#include <sophus/so3.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#import "TrackerHandler.h"
#import "ARDisplay.h"

using namespace vrlt;

@implementation VideoHandler
@synthesize controller;
@synthesize trackerHandler;
@synthesize useGyro;
@synthesize arDisplay;
@synthesize currentAttitude;

- (id)init
{
    if ( ( self = [super init] ) )
    {
        NSString *plistPath = [[NSBundle mainBundle] pathForResource:@"default_settings" ofType:@"plist"];
        NSDictionary *settings = [[NSDictionary alloc] initWithContentsOfFile:plistPath];

        useGyro = YES;
        
        distortedData = (unsigned char *)malloc(1280*720);
        undistortedData = (unsigned char *)malloc(1280*720);
        
        Eigen::Matrix3d mymat;
        mymat <<
        0,-1, 0,
        -1, 0, 0,
        0, 0,-1;
        gyroconversion = Sophus::SO3d( mymat );
        
        NSString *trackingVideoPath = [[settings objectForKey:@"trackingVideoPath"] retain];
        if ( [trackingVideoPath length] != 0 )
        {
            useGyro = NO;
                
            videoLoader = [[VideoLoader alloc] initFromFile:trackingVideoPath];
            videoLoader.delegate = self;
        }
        
        NSString *trackingModelPath = [[settings objectForKey:@"trackingModelPath"] retain];
        NSLog( @"opening tracking model: %@", trackingModelPath );
        
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsDirectory = [paths objectAtIndex:0];

        NSString *prefix = [[NSString alloc] initWithFormat:@"%@/%@",documentsDirectory,trackingModelPath];
        [trackingModelPath release];
        
        NSNumber *scaleNumber = [[settings objectForKey:@"adjustmentScale"] retain];
        int scale = [scaleNumber intValue];
        [scaleNumber release];

        NSNumber *shiftNumber = [[settings objectForKey:@"adjustmentShift"] retain];
        int shift = [shiftNumber intValue];
        [shiftNumber release];
        
        self.trackerHandler = [[TrackerHandler alloc] initWithPrefix:prefix scale:scale shift:shift];
        trackerHandler.videoHandler = self;
        [prefix release];
        
        lowRotThresh = 2;
        highRotThresh = 10;
        stateCount = 0;
        countThresh = 10;
        highMotion = NO;

        gyroQueue = [[NSOperationQueue alloc] init];
        poseLock = [[NSLock alloc] init];
        
        motionManager = [[CMMotionManager alloc] init];
        motionManager.gyroUpdateInterval = 0.01;
        motionManager.deviceMotionUpdateInterval = 0.01;
        [motionManager startDeviceMotionUpdates];
        
        self.arDisplay = [[ARDisplay alloc] initWithNode:trackerHandler.root];
        
        if ( [[settings objectForKey:@"displayModelPath"] length] != 0 )
        {
            NSString *displayModelPath = [[NSString stringWithFormat:@"%@/models/%@",documentsDirectory,[settings objectForKey:@"displayModelPath"]] retain];
            NSString *displayModelFilename = [[settings objectForKey:@"displayModelFilename"] retain];

            [arDisplay setOBJModelFromPath:displayModelPath filename:displayModelFilename];
            [arDisplay setModelScale:[settings objectForKey:@"modelScale"]];
            [arDisplay setModelTranslation:[settings objectForKey:@"modelTranslation"]];
            [arDisplay setModelRotation:[settings objectForKey:@"modelRotation"]];
            [arDisplay setLightDirection:[settings objectForKey:@"lightDirection"]];
            [arDisplay setPlane:[settings objectForKey:@"plane"]];
        
            [displayModelFilename release];
            [displayModelPath release];
        }
        
        [settings release];
        
        if ( videoLoader ) [videoLoader start];
    }
    
    return self;
}

- (void)dealloc
{
    free(distortedData);
    free(undistortedData);
    
    [undistorter release];
    [motionManager release];
    [gyroQueue release];
    [trackerHandler release];
    [arDisplay release];
    
    [super dealloc];
}

- (void)addObjectAtPoint:(CGPoint)point
{
    if ( !trackerHandler.tracked ) return;
    
    [arDisplay addModelInstance];
    [arDisplay setModelPlanePoint:point withPose:trackerHandler.node->pose andCalibration:trackerHandler.calibration];
}

- (void)moveObjectToPoint:(CGPoint)point
{
    if ( !trackerHandler.tracked ) return;

    [arDisplay setModelPlanePoint:point withPose:trackerHandler.node->pose andCalibration:trackerHandler.calibration];
}

- (void)startRecording
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];

    NSString *path = [NSString stringWithFormat:@"%@/%@.mp4",documentsDirectory,[[NSDate date] description]];
    NSLog( @"path: %@", path );
    [self.arDisplay startRecording:[NSURL fileURLWithPath:path]];
}

- (void)endRecording
{
    [self.arDisplay stopRecording];
}

- (void)localize
{
    trackerHandler.needLocalize = YES;
}

- (void)reportStatus:(NSString *)string
{
    controller.statusLabel.text = string;
}

- (void)reportTiming:(NSString *)string
{
    controller.timingLabel.text = string;
}

- (void)reportMotion
{
    controller.motionLabel.text = [NSString stringWithFormat:@"state: %@  stable: %@",
                                   (highMotion)?@"high":@"low",
                                   (stateCount>countThresh)?@"yes":@"no"];
}

- (void)reportFPS:(double)fps
{
    controller.fpsLabel.text = [NSString stringWithFormat:@"fps: %g", fps];
}

- (void)processMotion:(CMDeviceMotion*)motion
{
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    
    
    // convert attitude to rotation
    CMRotationMatrix attitude = motion.attitude.rotationMatrix;
    Eigen::Matrix3d Rmat;
    Rmat <<
    attitude.m11, attitude.m12, attitude.m13,
    attitude.m21, attitude.m22, attitude.m23,
    attitude.m31, attitude.m32, attitude.m33;
    Sophus::SO3d R( Rmat );
    
    currentAttitude = gyroconversion * R * gyroconversion;
    


    
    
    // check rotation rate
    if ( lastMotionTime == 0 )
    {
        lastMotionTime = [[NSDate date] timeIntervalSinceReferenceDate];
    }
    double thisMotionTime = [[NSDate date] timeIntervalSinceReferenceDate];
    double elapsed = thisMotionTime - lastMotionTime;
    lastMotionTime = thisMotionTime;

    double x = motion.rotationRate.x * elapsed;
    double y = motion.rotationRate.y * elapsed;
    double z = motion.rotationRate.z * elapsed;
    
    Sophus::SO3d rotrate( Sophus::SO3d::exp( makeVector( 0., 0., z ) ) * Sophus::SO3d::exp( makeVector( 0., y, 0. ) ) * Sophus::SO3d::exp( makeVector( x, 0., 0. ) ) );
    double mag = rotrate.log().norm() * 180. / M_PI;

    if ( mag >= highRotThresh && !highMotion )
    {
        highMotion = YES;
        stateCount = 0;
    }
    if ( mag <= lowRotThresh && highMotion )
    {
        highMotion = NO;
        stateCount = 0;
    }
    stateCount++;
    [self reportMotion];
    
    
    [pool release];
}

- (void)processFrame:(unsigned char *)grayData
{
    [trackerHandler setGrayData:grayData];

    CMDeviceMotion *deviceMotion = [motionManager.deviceMotion retain];
    if ( deviceMotion ) {
        lastAttitude = currentAttitude;
        [self processMotion:deviceMotion];
        [deviceMotion release];
    }
    
    if ( trackerHandler.needLocalize && !highMotion && stateCount>countThresh )
    {
        trackerHandler.needLocalize = NO;
        
        LocalizationRequest *request = [[LocalizationRequest alloc] initWithImageData:grayData attitude:currentAttitude];
        [NSThread detachNewThreadSelector:@selector(requestLocalization:) toTarget:trackerHandler withObject:request];
    }
    
    [poseLock lock];
    
    if ( self.useGyro )
    {
        trackerHandler.node->pose.so3() = ( currentAttitude * lastAttitude.inverse() ) * trackerHandler.node->pose.so3();
        trackerHandler.node->pose.translation() = ( currentAttitude * lastAttitude.inverse() ) * trackerHandler.node->pose.translation();
    }

    [trackerHandler process];
    [self reportStatus:[NSString stringWithFormat:@"%d frames in cache", [trackerHandler numFramesInCache]]];
    
    arDisplay.tracked = trackerHandler.tracked;
    [arDisplay renderWithPose:trackerHandler.node->pose];

    [poseLock unlock];    
}

- (void)processFrame:(unsigned char *)grayData withPose:(Sophus::SE3d)pose
{
    [poseLock lock];
    if ( trackerHandler.needLocalize )
    {
        trackerHandler.node->pose = pose;
        trackerHandler.needLocalize = NO;
    }
    [poseLock unlock];
    
    [self processFrame:grayData];
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

    [arDisplay setImageBuffer:imageBuffer];
    
    CVPixelBufferLockBaseAddress(imageBuffer,0);
    unsigned char *YData = (unsigned char *)CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0 );
    [self processFrame:YData];
    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
}

@end
