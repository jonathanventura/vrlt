/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: VideoView.m
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import "VideoView.h"
#import <QuartzCore/QuartzCore.h>

@implementation VideoView

- (void)awakeFromNib
{
    [super awakeFromNib];

    grayColorSpace = CGColorSpaceCreateDeviceGray();
    dataProvider = NULL;
}

/*
// Only override drawRect: if you perform custom drawing.
// An empty implementation adversely affects performance during animation.
- (void)drawRect:(CGRect)rect
{
    // Drawing code
}
*/

- (void)setData:(const unsigned char*)graydata
{
    CGDataProviderRelease( dataProvider );
    dataProvider = CGDataProviderCreateWithData( NULL, graydata, 1280*720, NULL );
}

- (void)refreshImage
{
    CGImageRef imageref = CGImageCreate( 1280, 720, 8, 8, 1280,
                                        grayColorSpace, kCGImageAlphaNone | kCGBitmapByteOrderDefault,
                                        dataProvider, NULL, true, kCGRenderingIntentDefault );
    self.layer.contents = (id)imageref;
    CGImageRelease(imageref);
}

- (void)dealloc
{
    CGColorSpaceRelease( grayColorSpace );
    [super dealloc];
}

@end
