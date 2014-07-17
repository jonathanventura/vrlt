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

@class MainViewController;

#import <CoreMotion/CoreMotion.h>
#import <CoreLocation/CoreLocation.h>

@interface VideoHandler : NSObject<AVCaptureVideoDataOutputSampleBufferDelegate> {
    MainViewController *controller;
    int frameRate;
    
    BOOL isRecording;
    
    NSString *xmlPath;
    NSString *imagesPath;
    NSString *imagesDirectory;
    FILE *xmlFile;
    
    NSDate *startDate;
    int frameno;
    double timestamp;
    
    CMMotionManager *motionManager;
    double rotationRate[3];
    double gravity[3];

    CLLocationManager *locationManager;
    
    double altitude;
    double verticalAccuracy;
    
    double latitude;
    double longitude;
    double horizontalAccuracy;
    
    double heading;
    double headingAccuracy;
    
    AVAssetWriter *assetWriter;
    AVAssetWriterInput *assetWriterInput;
    AVAssetWriterInputPixelBufferAdaptor *assetWriterInputAdaptor;
    CMTime assetNextTime;
}
- (void)setDeviceOrientation:(UIInterfaceOrientation)orientation;
- (void)startRecordingWithTitle:(NSString *)title;
- (void)endRecording;
@property (nonatomic,retain) MainViewController *controller;
@property (nonatomic,assign) int frameRate;
@end
