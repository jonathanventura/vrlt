//
//  VideoView.m
//  iOSTracker
//
//  Created by Jonathan Ventura on 8/20/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#import "VideoView.h"
#import <QuartzCore/QuartzCore.h>

@implementation VideoView

- (void)awakeFromNib
{
    [super awakeFromNib];

    grayColorSpace = CGColorSpaceCreateDeviceGray();
    dataProvider = NULL;
}

- (void)setData:(const unsigned char*)grayData
{
    CGDataProviderRelease( dataProvider );
    dataProvider = CGDataProviderCreateWithData( NULL, grayData, 1280*720, NULL );
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
