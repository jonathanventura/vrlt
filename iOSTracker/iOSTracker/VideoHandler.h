/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: VideoHandler.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

@class TrackerHandler;
@class MainViewController;
@class UnDistorter;
@class ARDisplay;

#import <CoreMotion/CoreMotion.h>

#include <Eigen/Core>
#include <Sophus/so3.hpp>

@interface VideoHandler : NSObject<AVCaptureVideoDataOutputSampleBufferDelegate> {
    MainViewController *controller;
    
    NSOperationQueue *gyroQueue;
    NSLock *poseLock;
    
    unsigned char *distortedData;
    unsigned char *undistortedData;

    UnDistorter *undistorter;
    TrackerHandler *trackerHandler;
    
    CMMotionManager *motionManager;
    double lowRotThresh;
    double highRotThresh;
    BOOL highMotion;
    int stateCount; // num frames in state
    int countThresh;    // num frames before state is good
    
    double lastMotionTime;
    Sophus::SO3d gyroconversion;
    
    BOOL useGyro;
    BOOL freeze;
    
    ARDisplay *arDisplay;
    
    Sophus::SO3d lastAttitude;
    Sophus::SO3d currentAttitude;
}
- (void)processMotion:(CMDeviceMotion*)motion;
- (void)localize;
- (void)reportMotion;
- (void)reportStatus:(NSString*)string;
- (void)reportTiming:(NSString*)string;
- (void)reportFPS:(double)fps;
- (void)startRecording;
- (void)endRecording;

- (void)addObjectAtPoint:(CGPoint)point;
- (void)moveObjectToPoint:(CGPoint)point;

@property (nonatomic,retain) MainViewController *controller;
@property (nonatomic,assign) TrackerHandler *trackerHandler;
@property (nonatomic,assign) BOOL useGyro;
@property (nonatomic,retain) ARDisplay *arDisplay;
@property (nonatomic,assign) Sophus::SO3d currentAttitude;
@end
