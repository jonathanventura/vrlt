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

@interface MainViewController : UIViewController<UITextFieldDelegate,UIAlertViewDelegate,AVCaptureFileOutputRecordingDelegate> {
    AVCaptureDevice *captureDevice;
    AVCaptureSession *captureSession;
    AVCaptureMovieFileOutput *movieOutput;
    AVCaptureConnection *movieConnection;
    AVCaptureVideoDataOutput *captureOutput;
    AVCaptureConnection *captureConnection;
    AVCaptureVideoPreviewLayer *previewLayer;
    
    VideoHandler *videoHandler;
    
    IBOutlet UIView *videoView;
    IBOutlet UIBarButtonItem *recordButton;
    IBOutlet UIActivityIndicatorView *loadingIndicator;

    UIAlertView *alertView;
}
@property (nonatomic,retain) UIAlertView *alertView;
@property (nonatomic,retain) AVCaptureVideoPreviewLayer *previewLayer;
@property (nonatomic,retain) AVCaptureDevice *captureDevice;
- (float)getRotationForOrientation:(UIInterfaceOrientation)interfaceOrientation;
- (IBAction)recordButton:(id)sender;
- (void)reset;
@end
