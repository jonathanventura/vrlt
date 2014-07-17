/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: MainViewController.m
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import "MainViewController.h"
#import "VideoHandler.h"
#import <CoreGraphics/CGAffineTransform.h>

@implementation MainViewController
@synthesize alertView;
@synthesize captureDevice;
@synthesize previewLayer;

- (void)dealloc
{
    [super dealloc];
}

- (void)didReceiveMemoryWarning
{
    // Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];
    
    // Release any cached data, images, etc that aren't in use.
}

- (void)setupTracker:(id)sender
{
    videoHandler = [[VideoHandler alloc] init];
    videoHandler.controller = self;
    [videoHandler setDeviceOrientation:self.interfaceOrientation];

    NSError *err;
    
    // create capture session
    // set for high-quality photos
    captureSession = [[AVCaptureSession alloc] init];
    captureSession.sessionPreset = AVCaptureSessionPresetInputPriority;
    
    // get camera
    self.captureDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    
    AVCaptureDeviceFormat *bestFormat = nil;
    AVFrameRateRange *bestFrameRateRange = nil;
    for ( AVCaptureDeviceFormat *format in [self.captureDevice formats] ) {
        for ( AVFrameRateRange *range in format.videoSupportedFrameRateRanges ) {
            FourCharCode codec = CMFormatDescriptionGetMediaSubType(format.formatDescription);
            unsigned char c4 = (codec >> 0) & (0xFF);
            if ( c4 != 'f' ) continue;
            NSLog( @"%d x %d x %d", CMVideoFormatDescriptionGetDimensions(format.formatDescription).width, CMVideoFormatDescriptionGetDimensions(format.formatDescription).height, range.minFrameDuration.timescale );
            if ( CMVideoFormatDescriptionGetDimensions(format.formatDescription).width != 1280 ) continue;
            if ( range.minFrameDuration.timescale != 30 ) continue;
            bestFormat = format;
            bestFrameRateRange = range;
        }
    }
    NSLog( @"max frame rate: %g", bestFrameRateRange.maxFrameRate );
    if ( bestFormat ) {
        if ( [self.captureDevice lockForConfiguration:NULL] == YES ) {
            NSLog( @"setting format" );
            self.captureDevice.activeFormat = bestFormat;
            self.captureDevice.activeVideoMinFrameDuration = bestFrameRateRange.minFrameDuration;
            self.captureDevice.activeVideoMaxFrameDuration = bestFrameRateRange.minFrameDuration;
            [self.captureDevice unlockForConfiguration];
        } else {
            NSLog( @"could not set format" );
        }
    }
    
    NSLog( @"min: %g, max: %g",
          (double)self.captureDevice.activeVideoMinFrameDuration.value/self.captureDevice.activeVideoMinFrameDuration.timescale,
          (double)self.captureDevice.activeVideoMaxFrameDuration.value/self.captureDevice.activeVideoMaxFrameDuration.timescale
          );

    videoHandler.frameRate = bestFrameRateRange.maxFrameRate;

    for ( AVFrameRateRange *range in self.captureDevice.activeFormat.videoSupportedFrameRateRanges ) {
        NSLog(@"frame rate min/max: %g / %g", range.minFrameRate,range.maxFrameRate);
    }
    
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:captureDevice error:&err];
    [captureSession addInput:input];
    
    // create image output
    captureOutput = [[AVCaptureVideoDataOutput alloc] init];
    captureOutput.alwaysDiscardsLateVideoFrames = NO;
    
    [captureOutput setSampleBufferDelegate:videoHandler queue:dispatch_get_main_queue()];
    [captureSession addOutput:captureOutput];

    movieOutput = [[AVCaptureMovieFileOutput alloc] init];
    [captureSession addOutput:movieOutput];
    
    movieConnection = [movieOutput.connections objectAtIndex:0];
    [movieConnection setVideoOrientation:AVCaptureVideoOrientationLandscapeRight];

    // get connection between input and output
    captureConnection = [captureOutput.connections objectAtIndex:0];
    
    // set up preview layer
    self.previewLayer = [AVCaptureVideoPreviewLayer layerWithSession:captureSession];
    self.previewLayer.videoGravity = AVLayerVideoGravityResizeAspect;
    [videoView.layer addSublayer:previewLayer];
    float rotation = [self getRotationForOrientation:self.interfaceOrientation];
    self.previewLayer.transform = CATransform3DMakeRotation(rotation, 0, 0, 1.0);
    self.previewLayer.frame = videoView.frame;
    

    // start capture session
    [captureSession startRunning];
    
    [loadingIndicator stopAnimating];
}

- (float)getRotationForOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    float rotation;
    switch ( interfaceOrientation ) {
        case UIInterfaceOrientationPortrait:
            rotation = 0;
            break;
            
        case UIInterfaceOrientationLandscapeLeft:
            rotation = M_PI/2;
            break;
            
        case UIInterfaceOrientationLandscapeRight:
            rotation = -M_PI/2;
            break;
            
        case UIInterfaceOrientationPortraitUpsideDown:
            rotation = M_PI;
            break;
    }
    return rotation;
}

#pragma mark - View lifecycle

// Implement viewDidLoad to do additional setup after loading the view, typically from a nib.
- (void)viewDidLoad
{
    [super viewDidLoad];
    
    [loadingIndicator startAnimating];
    
    [self performSelector:@selector(setupTracker:) onThread:[NSThread currentThread] withObject:nil waitUntilDone:NO];
}

- (void)viewDidUnload
{
    [super viewDidUnload];
    // Release any retained subviews of the main view.
    // e.g. self.myOutlet = nil;
    
    [videoHandler endRecording];
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    return [recordButton.title isEqualToString:@"Start"];
}

- (void)willRotateToInterfaceOrientation:(UIInterfaceOrientation)toInterfaceOrientation duration:(NSTimeInterval)duration
{
    float rotation = [self getRotationForOrientation:toInterfaceOrientation];
    
    [UIView animateWithDuration:duration animations:^{
        self.previewLayer.transform = CATransform3DMakeRotation(rotation, 0, 0, 1.0);
    }];
}

- (void)didRotateFromInterfaceOrientation:(UIInterfaceOrientation)fromInterfaceOrientation
{
    [videoHandler setDeviceOrientation:self.interfaceOrientation];
    CGRect bounds = videoView.frame;
    self.previewLayer.frame = bounds;
}

- (BOOL)textFieldShouldReturn:(UITextField *)textField
{
    [self.alertView dismissWithClickedButtonIndex:1 animated:YES];
    return YES;
}

- (void)alertView:(UIAlertView *)alertView didDismissWithButtonIndex:(NSInteger)buttonIndex
{
    if ( buttonIndex == 1 ) {
//        [videoHandler startRecordingWithTitle:[self.alertView textFieldAtIndex:0].text];
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsDirectory = [paths objectAtIndex:0];
        NSString *videopath = [NSString stringWithFormat:@"%@/%@.mp4",documentsDirectory,[self.alertView textFieldAtIndex:0].text];
        [movieOutput startRecordingToOutputFileURL:[NSURL fileURLWithPath:videopath] recordingDelegate:self];
    } else {
        [self reset];
    }
    [loadingIndicator stopAnimating];
}

- (IBAction)recordButton:(id)sender
{
    if ( [recordButton.title isEqualToString:@"Start"] ) {
        if ( [self.captureDevice lockForConfiguration:NULL] == YES ) {
            self.captureDevice.focusMode = AVCaptureFocusModeLocked;
            self.captureDevice.exposureMode = AVCaptureExposureModeLocked;
            [self.captureDevice unlockForConfiguration];
        }
        recordButton.title = @"Stop";
        [loadingIndicator startAnimating];
        self.alertView = [[UIAlertView alloc] initWithTitle:@"Start New Recording" message:@"Enter the recording name:" delegate:self cancelButtonTitle:@"Cancel" otherButtonTitles:@"OK",nil];
        self.alertView.alertViewStyle = UIAlertViewStylePlainTextInput;
        [self.alertView textFieldAtIndex:0].text = @"Untitled";
        [self.alertView textFieldAtIndex:0].delegate = self;
        [self.alertView show];
    }
    else
    {
        [movieOutput stopRecording];
//        [videoHandler endRecording];
    }
}

- (void)reset
{
    if ( [self.captureDevice lockForConfiguration:NULL] == YES ) {
        self.captureDevice.focusMode = AVCaptureFocusModeContinuousAutoFocus;
        self.captureDevice.exposureMode = AVCaptureExposureModeContinuousAutoExposure;
        [self.captureDevice unlockForConfiguration];
    }
    recordButton.title = @"Start";
}

- (void)captureOutput:(AVCaptureFileOutput *)captureOutput didStartRecordingToOutputFileAtURL:(NSURL *)fileURL fromConnections:(NSArray *)connections
{
    
}

- (void)captureOutput:(AVCaptureFileOutput *)captureOutput didFinishRecordingToOutputFileAtURL:(NSURL *)outputFileURL fromConnections:(NSArray *)connections error:(NSError *)error
{
    [self reset];
}

@end
