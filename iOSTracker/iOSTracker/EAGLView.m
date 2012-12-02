/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: EAGLView.m
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import "EAGLView.h"

#import <QuartzCore/CAEAGLLayer.h>

#import "MainViewController.h"

@implementation EAGLView
@synthesize mainViewController;

+ (Class) layerClass
{
    return [CAEAGLLayer class];
}

- (id) initWithCoder:(NSCoder*)coder
{    
    if ((self = [super initWithCoder:coder]))
	{
        // Get the layer
        CAEAGLLayer *eaglLayer = (CAEAGLLayer *)self.layer;
        
        eaglLayer.opaque = TRUE;
        eaglLayer.drawableProperties = [NSDictionary dictionaryWithObjectsAndKeys:
                                        [NSNumber numberWithBool:FALSE], kEAGLDrawablePropertyRetainedBacking, kEAGLColorFormatRGBA8, kEAGLDrawablePropertyColorFormat, nil];
    }
	
    return self;
}

- (void) layoutSubviews
{
	[mainViewController resizeFromLayer:(CAEAGLLayer*)self.layer];
}

@end
