//
//  ImageWriter.h
//  iOSRecorder
//
//  Created by Jonathan Ventura on 11/18/11.
//  Copyright (c) 2011 __MyCompanyName__. All rights reserved.
//

#import <Foundation/Foundation.h>

#include <dispatch/dispatch.h>

@interface ImageWriter : NSObject
{
    dispatch_queue_t queue;
    dispatch_group_t group;
}
- (void)enqueueImage:(UIImage *)image path:(NSString *)path;
- (void)finishQueue;
@end
