//
//  VideoLoader.h
//  iOSTracker
//
//  Created by Jonathan Ventura on 7/16/14.
//
//

#import <Foundation/Foundation.h>

#include <Sophus/se3.hpp>

@protocol VideoLoaderDelegate;

@interface VideoLoader : NSObject

- (id)initFromFile:(NSString*)path;
- (void)start;

@property (nonatomic,retain) NSObject<VideoLoaderDelegate> *delegate;
@end

@protocol VideoLoaderDelegate<NSObject>

- (void)processFrame:(unsigned char *)grayData;
- (void)processFrame:(unsigned char *)grayData withPose:(Sophus::SE3d)pose;

@end