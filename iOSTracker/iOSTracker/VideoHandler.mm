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

#include <TooN/so3.h>

#include <cvd/image.h>
#include <cvd/image_convert.h>

#import "TrackerHandler.h"
#import "ARDisplay.h"

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
        
        TooN::Matrix<3> mymat;
        mymat[0] = TooN::makeVector( 0,-1, 0 );
        mymat[1] = TooN::makeVector(-1, 0, 0 );
        mymat[2] = TooN::makeVector( 0, 0,-1 );
        gyroconversion = TooN::SO3<>( mymat );
        
        NSString *trackingModelPath = [[settings objectForKey:@"trackingModelPath"] retain];
        NSLog( @"opening tracking model: %@", trackingModelPath );
        
        NSString *prefix = [[NSString alloc] initWithFormat:@"%@/%@",[[NSBundle mainBundle] bundlePath],trackingModelPath];
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
        
        NSString *displayModelPath = [[NSString stringWithFormat:@"%@/models/%@",[[NSBundle mainBundle] bundlePath],[settings objectForKey:@"displayModelPath"]] retain];
        NSString *displayModelFilename = [[settings objectForKey:@"displayModelFilename"] retain];
        
        [arDisplay setOBJModelFromPath:displayModelPath filename:displayModelFilename];
        [arDisplay setModelScale:[settings objectForKey:@"modelScale"]];
        [arDisplay setModelTranslation:[settings objectForKey:@"modelTranslation"]];
        [arDisplay setModelRotation:[settings objectForKey:@"modelRotation"]];
        [arDisplay setLightDirection:[settings objectForKey:@"lightDirection"]];
        [arDisplay setPlane:[settings objectForKey:@"plane"]];
        
        [displayModelFilename release];
        [displayModelPath release];
        
        [settings release];
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
    TooN::Matrix<3> Rmat;
    Rmat(0,0) = attitude.m11;   Rmat(0,1) = attitude.m12;   Rmat(0,2) = attitude.m13;
    Rmat(1,0) = attitude.m21;   Rmat(1,1) = attitude.m22;   Rmat(1,2) = attitude.m23;
    Rmat(2,0) = attitude.m31;   Rmat(2,1) = attitude.m32;   Rmat(2,2) = attitude.m33;
    TooN::SO3<> R( Rmat );
    
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
    
    TooN::SO3<> rotrate( TooN::SO3<>( TooN::makeVector( 0, 0, z ) ) * TooN::SO3<>( TooN::makeVector( 0, y, 0 ) ) * TooN::SO3<>( TooN::makeVector( x, 0, 0 ) ) );
    double mag = TooN::norm( rotrate.ln() ) * 180. / M_PI;

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
        trackerHandler.node->pose = currentAttitude * lastAttitude.inverse() * trackerHandler.node->pose;
    }

    [trackerHandler process];
    [self reportStatus:[NSString stringWithFormat:@"%d frames in cache", [trackerHandler numFramesInCache]]];
    
    arDisplay.tracked = trackerHandler.tracked;
    [arDisplay renderWithPose:trackerHandler.node->pose];

    [poseLock unlock];    
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
