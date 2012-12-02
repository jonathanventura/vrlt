/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: TrackerHandler.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
//#include <cvd/timer.h>

#include <MultiView/multiview.h>
#include <PatchTracker/tracker.h>
#include <PatchTracker/robustlsq.h>
#include <LocalizerClient/client.h>

#include <ImageCache/imagecache.h>

#include <CoreMotion/CoreMotion.h>

@class VideoHandler;

@interface TrackerHandler : NSObject
{
//    CVD::cvd_timer fpsTimer;
    NSDate *fpsTimer;
    int nFpsTimerFrames;
    
    VideoHandler *videoHandler;
    CVD::ImageRef imsize;
    int maxnumpoints;
    int minnumpoints;
    float minratio;
    int ntimeslost;
    vrlt::Calibration *calibration;
    vrlt::Camera *camera;
    vrlt::Node *root;
    vrlt::Node *node;
    
    vrlt::ImageCache *imagecache;
    NSLock *imagecachelock;
    
    vrlt::Tracker *tracker;
    vrlt::Reconstruction *r;
    
    BOOL doTrack;
    BOOL tracked;

    BOOL needLocalize;
    vrlt::LocalizationClient *localizer;
    NSLock *localizerlock;
    NSMutableArray *localizerResponses;
    NSLock *localizerResponsesLock;
    
    vrlt::Camera *newcamera;
    
    vrlt::RobustLeastSq *robustlsq;
//    tag::KalmanFilter< tag::ConstantPosition::State, tag::ConstantPosition::Model > kalman;

    int scale;
    int shift;
    unsigned char *data2;
    unsigned char *data4;
    unsigned char *data8;
    unsigned char *packedData;
}
- (void)setGrayData:(unsigned char *)grayData;
- (id)initWithPrefix:(NSString*)prefix scale:(int)theScale shift:(int)theShift;
- (int)numFramesInCache;
- (void)process;
- (void)requestLocalization:(id)arg;
@property (assign) int maxnumpoints;
@property (assign) BOOL needLocalize;
@property (nonatomic,retain) VideoHandler *videoHandler;
@property (assign) vrlt::Calibration *calibration;
@property (assign) vrlt::Node *root;
@property (assign) vrlt::Node *node;
@property (assign) BOOL doTrack;
@property (assign) BOOL tracked;
@end

@interface LocalizationRequest : NSObject
{
    unsigned char *imagedata;
    double *posedata;
    TooN::SO3<> attitude;
}
- (id)initWithImageData:(const unsigned char *)data attitude:(TooN::SO3<>)att;
@property (assign) unsigned char *imagedata;
@property (assign) double *posedata;
@property (assign) TooN::SO3<> attitude;
@end
