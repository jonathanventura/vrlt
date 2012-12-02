//
//  ImageWriter.m
//  iOSRecorder
//
//  Created by Jonathan Ventura on 11/18/11.
//  Copyright (c) 2011 __MyCompanyName__. All rights reserved.
//

#import "ImageWriter.h"

@interface WriteData : NSObject
{
    UIImage *image;
    NSString *path;
}
@property (nonatomic,retain) UIImage *image;
@property (nonatomic,retain) NSString *path;
@end

@implementation WriteData
@synthesize image,path;
- (void)dealloc
{
    [image release];
    [path release];
    
    [super dealloc];
}
@end

void writeFunc( void *context );

@implementation ImageWriter

- (id)init
{
    if ( ( self = [super init] ) )
    {
        queue = dispatch_queue_create("myqueue", NULL);
        group = dispatch_group_create();
    }
    
    return self;
}

- (void)dealloc
{
    dispatch_release( queue );
    dispatch_release( group );
 
    [super dealloc];
}

void writeFunc( void *context )
{
    WriteData *writedata = (WriteData*)context;
    [UIImageJPEGRepresentation( writedata.image, 1.0 ) writeToFile:writedata.path atomically:YES];
    [writedata release];
}

- (void)finishQueue
{
    dispatch_group_wait( group, DISPATCH_TIME_FOREVER );
}

- (void)enqueueImage:(UIImage *)image path:(NSString *)path
{
    WriteData *mydata = [[WriteData alloc] init];
    mydata.image = image;
    mydata.path = path;
    dispatch_group_async_f( group, queue, mydata, writeFunc );
}

@end
