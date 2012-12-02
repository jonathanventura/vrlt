//
//  VideoView.h
//  iOSTracker
//
//  Created by Jonathan Ventura on 8/20/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#import <UIKit/UIKit.h>


@interface VideoView : UIView {
    CGColorSpaceRef grayColorSpace;
    CGDataProviderRef dataProvider;
}
- (void)setData:(const unsigned char*)grayData;
- (void)refreshImage;
@end
