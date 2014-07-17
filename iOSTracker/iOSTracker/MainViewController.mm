/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: MainViewController.mm
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <TargetConditionals.h>

#import "MainViewController.h"
#import "TrackerHandler.h"
#import "VideoHandler.h"
#import "VideoView.h"
#import "EAGLView.h"
#import "ARDisplay.h"

@implementation MainViewController
@synthesize loadingIndicator;
@synthesize captureDevice;
@synthesize motionLabel;
@synthesize statusLabel;
@synthesize timingLabel;
@synthesize fpsLabel;
@synthesize videoView;
@synthesize sharegroup;

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
    videoView.mainViewController = self;
    
#if !(TARGET_IPHONE_SIMULATOR)
    NSError *err;
    
    // create capture session
    // set for high-quality photos
    captureSession = [[AVCaptureSession alloc] init];
    captureSession.sessionPreset = AVCaptureSessionPreset1280x720;
    
    // get camera
    self.captureDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:captureDevice error:&err];
    [captureSession addInput:input];
    
    // create image output
    captureOutput = [[AVCaptureVideoDataOutput alloc] init];
    [captureOutput setSampleBufferDelegate:videoHandler queue:dispatch_get_main_queue()];
    [captureSession addOutput:captureOutput];
    
    // get connection between input and output
    captureConnection = [captureOutput.connections objectAtIndex:0];
    
    // start capture session
    [captureSession startRunning];
#endif
    
    [loadingIndicator stopAnimating];
    [self.videoView setNeedsLayout];
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
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    // Return YES for supported orientations
    return (interfaceOrientation == UIInterfaceOrientationLandscapeRight);
}

- (void)refreshImage
{
    //[videoView refreshImage];
}

- (IBAction)localizeButton:(id)sender
{
    [videoHandler localize];
}

- (IBAction)gyroButton:(id)sender
{
    videoHandler.useGyro = gyroSwitch.on;
}

- (IBAction)recordButton:(id)sender
{
    if ( recordSwitch.on ) [videoHandler startRecording];
    else [videoHandler endRecording];
}

- (IBAction)pointsButton:(id)sender
{
    videoHandler.arDisplay.showPoints = pointsSwitch.on;
}

- (IBAction)pointsSliderChanged:(id)sender
{
    videoHandler.trackerHandler.maxnumpoints = floor( pointsSlider.value );
    pointsLabel.text = [NSString stringWithFormat:@"%d",videoHandler.trackerHandler.maxnumpoints];
}

- (void)resizeFromLayer:(CAEAGLLayer*)layer
{
    [videoHandler.arDisplay resizeFromLayer:layer];
}

#pragma mark - Gestures

- (BOOL)gestureRecognizer:(UIGestureRecognizer *)gestureRecognizer shouldReceiveTouch:(UITouch *)touch
{
    return ! ( [[touch view] isKindOfClass:[UISwitch class]] );
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    if ( [touches count] > 1 ) return;
    
    UITouch *touch = [touches anyObject];
    CGPoint point = [touch locationInView:self.videoView];
    
    [videoHandler addObjectAtPoint:point];
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    if ( [touches count] > 1 ) return;
    
    UITouch *touch = [touches anyObject];
    CGPoint point = [touch locationInView:self.videoView];
    
    [videoHandler moveObjectToPoint:point];
}

@end
