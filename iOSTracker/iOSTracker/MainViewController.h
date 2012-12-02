/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: MainViewController.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import <UIKit/UIKit.h>

#import <AVFoundation/AVFoundation.h>

@class VideoHandler;
@class VideoView;
@class EAGLView;

@interface MainViewController : UIViewController {
    AVCaptureDevice *captureDevice;
    AVCaptureSession *captureSession;
    AVCaptureVideoDataOutput *captureOutput;
    AVCaptureConnection *captureConnection;
    
    IBOutlet UIActivityIndicatorView *loadingIndicator;
    
    IBOutlet EAGLView *videoView;
    IBOutlet UILabel *pointsLabel;
    IBOutlet UISlider *pointsSlider;
    IBOutlet UILabel *motionLabel;
    IBOutlet UILabel *statusLabel;
    IBOutlet UILabel *timingLabel;
    IBOutlet UILabel *fpsLabel;
    VideoHandler *videoHandler;
    
    IBOutlet UISwitch *pointsSwitch;
    IBOutlet UISwitch *gyroSwitch;
    IBOutlet UISwitch *recordSwitch;
    
    EAGLSharegroup *sharegroup;
}
@property (nonatomic,retain) UIView *videoView;
@property (nonatomic,retain) UIActivityIndicatorView *loadingIndicator;
@property (nonatomic,retain) AVCaptureDevice *captureDevice;
@property (nonatomic,retain) UILabel *motionLabel;
@property (nonatomic,retain) UILabel *statusLabel;
@property (nonatomic,retain) UILabel *timingLabel;
@property (nonatomic,retain) UILabel *fpsLabel;
@property (nonatomic,retain) EAGLSharegroup *sharegroup;
- (IBAction)localizeButton:(id)sender;
- (IBAction)gyroButton:(id)sender;
- (IBAction)recordButton:(id)sender;
- (IBAction)pointsSliderChanged:(id)sender;
- (IBAction)pointsButton:(id)sender;
- (void)refreshImage;
- (void)resizeFromLayer:(CAEAGLLayer*)layer;
@end
