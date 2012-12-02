/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: ARDisplay.mm
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#import "ARDisplay.h"

#include "LoadOBJ.h"
#include "GLModel.h"

#include "CheckGL.h"

#include "GLSLHelpers.h"

using namespace vrlt;

@implementation ARDisplay
@synthesize showPoints;
@synthesize tracked;

- (id)initWithNode:(Node*)_node
{
    if ( ( self = [super init] ) )
    {
        ready = NO;
        
        showPoints = YES;

        node = _node;
        numpoints = node->points.size();
        
        model = NULL;
        
        context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
        [EAGLContext setCurrentContext:context];
        
        // make shadow framebuffer
        glGenFramebuffers( 1, &shadowFramebuffer );
        glBindFramebuffer( GL_FRAMEBUFFER, shadowFramebuffer );
        checkGL( "make shadow framebuffer" );
        
        // make shadow color texture
        glGenTextures( 1, &shadowTexture );
        glBindTexture( GL_TEXTURE_2D, shadowTexture );
        checkGL( "make shadow texture" );
        
        // make framebuffer
        glGenFramebuffers(1, &framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        checkGL( "make framebuffer" );
        
        // make color render buffer
        glGenRenderbuffers(1, &colorRenderbuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, colorRenderbuffer);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorRenderbuffer);
        checkGL( "make color render buffer" );

        // make depth render buffer
        glGenRenderbuffers(1, &depthRenderbuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderbuffer);
        checkGL( "make depth render buffer" );

        // make vertex buffer objects
        pointDrawable.Create();
        imageDrawable.Create();
        flippedImageDrawable.Create();
        shadowImageDrawable.Create();
        checkGL( "make vbos" );
        
        // make texture
        glPixelStorei( GL_PACK_ALIGNMENT, 1 );
        glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );

        checkGL( "make texture" );
        
        // create shaders
        pointShader.Create();
        imageShader.Create();
        modelShader.Create();
        shadowShader.Create();
        shadowImageShader.Create();
        checkGL( "make shaders" );
        
        // load point data
        pointDrawable.AddAttrib( 0, 3 );
        ElementList::iterator it;
        for ( it = node->points.begin(); it != node->points.end(); it++ )
        {
            vrlt::Point *point = (vrlt::Point *)it->second;
            
            pointDrawable.AddElem( project( point->position ) );
        }
        pointDrawable.Commit();
        checkGL( "load point data" );
        
        // load image vertex data
        imageDrawable.AddAttrib( 0, 2 );
        imageDrawable.AddAttrib( 1, 2 );
        imageDrawable.AddElem( TooN::makeVector(-1,-1 ), TooN::makeVector( 0, 0 ) );
        imageDrawable.AddElem( TooN::makeVector( 1,-1 ), TooN::makeVector( 1, 0 ) );
        imageDrawable.AddElem( TooN::makeVector(-1, 1 ), TooN::makeVector( 0, 1 ) );
        imageDrawable.AddElem( TooN::makeVector( 1, 1 ), TooN::makeVector( 1, 1 ) );
        imageDrawable.Commit();
        checkGL( "load image vertex data" );

        // load image vertex data
        shadowImageDrawable.AddAttrib( 0, 2 );
        shadowImageDrawable.AddAttrib( 1, 2 );
        shadowImageDrawable.AddElem( TooN::makeVector(-1,-1 ), TooN::makeVector( 0, 0 ) );
        shadowImageDrawable.AddElem( TooN::makeVector( 1,-1 ), TooN::makeVector( 1, 0 ) );
        shadowImageDrawable.AddElem( TooN::makeVector(-1, 1 ), TooN::makeVector( 0, 1 ) );
        shadowImageDrawable.AddElem( TooN::makeVector( 1, 1 ), TooN::makeVector( 1, 1 ) );
        shadowImageDrawable.Commit();
        checkGL( "load shadow image vertex data" );

        // load flipped image vertex data
        flippedImageDrawable.AddAttrib( 0, 2 );
        flippedImageDrawable.AddAttrib( 1, 2 );
        flippedImageDrawable.AddElem( TooN::makeVector(-1,-1 ), TooN::makeVector( 0, 1 ) );
        flippedImageDrawable.AddElem( TooN::makeVector( 1,-1 ), TooN::makeVector( 1, 1 ) );
        flippedImageDrawable.AddElem( TooN::makeVector(-1, 1 ), TooN::makeVector( 0, 0 ) );
        flippedImageDrawable.AddElem( TooN::makeVector( 1, 1 ), TooN::makeVector( 1, 0 ) );
        flippedImageDrawable.Commit();
        checkGL( "load image vertex data" );

        glClearColor( 0.f, 0.f, 0.f, 0.f );
                
        shadowColor = TooN::makeVector( 0, 0, 0, 0.7 );
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
        
        // make texture cache
        NSDictionary *cacheAttr = [NSDictionary dictionaryWithObjectsAndKeys:
                                   [NSNumber numberWithInt:0], kCVOpenGLESTextureCacheMaximumTextureAgeKey,
                                   nil];
        CVOpenGLESTextureCacheCreate(kCFAllocatorDefault, (CFDictionaryRef)cacheAttr, context, NULL, &textureCache);
    }
    
    return self;
}

- (void)dealloc
{
    free(pointVertices);
    
    [assetWriterInput release];
    [assetWriterInputAdaptor release];

    [super dealloc];
}

- (BOOL)resizeFromLayer:(CAEAGLLayer *)layer
{	
    [EAGLContext setCurrentContext:context];

    // Allocate shadow texture
    glBindTexture( GL_TEXTURE_2D, shadowTexture );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, backingWidth, backingHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    
    // Attach to shadow framebuffer
    glBindFramebuffer( GL_FRAMEBUFFER, shadowFramebuffer );
    glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, shadowTexture, 0 );
    glBindFramebuffer( GL_FRAMEBUFFER, framebuffer );

    // Allocate color buffer backing based on the current layer size
    glBindRenderbuffer(GL_RENDERBUFFER, colorRenderbuffer);
    [context renderbufferStorage:GL_RENDERBUFFER fromDrawable:layer];
    
    // Get backing size
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &backingWidth);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &backingHeight);
    NSLog( @"gl view size: %d %d", backingWidth, backingHeight);

    // Allocate depth buffer backing based on the current layer size
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer);
    glRenderbufferStorage( GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, backingWidth, backingHeight );

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		NSLog(@"Failed to make complete framebuffer object %x", glCheckFramebufferStatus(GL_FRAMEBUFFER));
        return NO;
    }
    
    ready = YES;
    
    return YES;
}

- (void)setOBJModelFromPath:(NSString *)path filename:(NSString *)filename
{
    [EAGLContext setCurrentContext:context];
    
    if ( model ) delete model;
    
    model = LoadOBJ( [path UTF8String], [filename UTF8String] );
}

- (void)addModelInstance
{
    if ( !modelOffsets.empty() ) return;
    //modelOffsets.push_back( TooN::makeVector( 0, 0, 0 ) );
    //modelOffsets.push_back( TooN::makeVector( .3, 0, -.2 ) );
    modelOffsets.push_back( TooN::makeVector( 2.65334, 0, 0.260111 ) );
}

- (void)setModelPlanePoint:(CGPoint)loc withPose:(TooN::SE3<>)pose andCalibration:(vrlt::Calibration*)calibration;
{
    if ( modelOffsets.empty() ) return;
    //return;
    
    // intersect with ground plane
    TooN::Vector<4> ground_plane = plane;//TooN::makeVector( 0, -1, 0, 0 );
    
    TooN::Vector<2> screen_pos = TooN::makeVector( loc.x / 1024. * 1280., loc.y / 748. * 720. );
    
    TooN::Vector<3> origin = -( pose.get_rotation().inverse() * pose.get_translation() );
    TooN::Vector<3> ray = pose.get_rotation().inverse() * calibration->unproject( screen_pos );
    
    // N * ( x0 + r * t ) + D = 0
    // N * x0 + N * r * t + D = 0
    // t = ( - D - N * x0 ) / ( N * r )
    
    double t = ( - ground_plane[3] - ground_plane.slice<0,3>() * origin ) / ( ground_plane.slice<0,3>() * ray );
    TooN::Vector<3> point = origin + t * ray;
    
    NSLog( @"point: %g %g %g", point[0], point[1], point[2] );
    
    modelOffsets.back() = point;
}

- (void)setModelScale:(NSString *)valString
{
    double vals[3];
    sscanf( [valString UTF8String], "%lf %lf %lf", vals+0, vals+1, vals+2 );
    modelScale[0] = vals[0] * .5;
    modelScale[1] = vals[1] * .5;
    modelScale[2] = vals[2] * .5;
}

- (void)setModelTranslation:(NSString *)valString
{
    double vals[3];
    sscanf( [valString UTF8String], "%lf %lf %lf", vals+0, vals+1, vals+2 );
    modelTranslation[0] = vals[0];
    modelTranslation[1] = vals[1];
    modelTranslation[2] = vals[2];
}

- (void)setModelRotation:(NSString *)valString
{
    double vals[3];
    sscanf( [valString UTF8String], "%lf %lf %lf", vals+0, vals+1, vals+2 );
    modelRotation[0] = vals[0];
    modelRotation[1] = vals[1];
    modelRotation[2] = vals[2];
}

- (void)setLightDirection:(NSString *)valString
{
    double vals[3];
    sscanf( [valString UTF8String], "%lf %lf %lf", vals+0, vals+1, vals+2 );
    lightDirection[0] = vals[0];
    lightDirection[1] = vals[1];
    lightDirection[2] = vals[2];
    normalize( lightDirection );
}

- (void)setPlane:(NSString *)valString
{
    double vals[4];
    sscanf( [valString UTF8String], "%lf %lf %lf", vals+0, vals+1, vals+2 );
    plane[0] = vals[0];
    plane[1] = vals[1];
    plane[2] = vals[2];
    plane[3] = vals[3];
}

- (void)setImageBuffer:(CVImageBufferRef)imageBuffer
{
    if ( YTexture ) {
        CFRelease( YTexture );
        YTexture = NULL;
        YTexID = 0;
    }
    
    if ( CbCrTexture ) {
        CVPixelBufferRelease( CbCrTexture );
        CbCrTexture = NULL;
        CbCrTexID = 0;
    }
    
    CVOpenGLESTextureCacheFlush( textureCache, 0 );
    
    // create Y texture
    CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault, textureCache, imageBuffer,
                                                 NULL, // texture attributes
                                                 GL_TEXTURE_2D,
                                                 GL_LUMINANCE,
                                                 1280,
                                                 720,
                                                 GL_LUMINANCE,
                                                 GL_UNSIGNED_BYTE,
                                                 0,
                                                 &YTexture);
    YTexID = CVOpenGLESTextureGetName(YTexture);
    glBindTexture( GL_TEXTURE_2D, YTexID );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // create CbCr texture
    CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault, textureCache, imageBuffer,
                                                 NULL, // texture attributes
                                                 GL_TEXTURE_2D,
                                                 GL_LUMINANCE_ALPHA,
                                                 640,
                                                 360,
                                                 GL_LUMINANCE_ALPHA,
                                                 GL_UNSIGNED_BYTE,
                                                 1,
                                                 &CbCrTexture);
    CbCrTexID = CVOpenGLESTextureGetName(CbCrTexture);
    glBindTexture( GL_TEXTURE_2D, CbCrTexID );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

- (void)renderPoints
{
    pointShader.shaderProgram.Use();
    if ( tracked ) pointShader.SetColor( TooN::makeVector( 0, 1, 0 ) );
    else pointShader.SetColor( TooN::makeVector( 1, 0, 0 ) );
    
    // set up vertex buffer and shader
    pointDrawable.PushClientState();
    
    // render points
    pointDrawable.Draw( GL_POINTS );
    
    // take down vertex buffer and shader
    pointDrawable.PopClientState();
}

- (void)renderImage:(BOOL)flipped
{
    imageShader.shaderProgram.Use();
    
    if ( flipped ) {
        // set up vertex buffer and shader
        flippedImageDrawable.PushClientState();
        
        // render image
        flippedImageDrawable.Draw( GL_TRIANGLE_STRIP );
        
        // take down vertex buffer and shader
        flippedImageDrawable.PopClientState();
    } else {
        // set up vertex buffer and shader
        imageDrawable.PushClientState();
        
        // render image
        imageDrawable.Draw( GL_TRIANGLE_STRIP );
        
        // take down vertex buffer and shader
        imageDrawable.PopClientState();
    }
}

- (void)_renderWithPose:(TooN::SE3<>)pose flipped:(BOOL)flipped
{
    // render video image
    glActiveTexture( GL_TEXTURE1 );
    glBindTexture( GL_TEXTURE_2D, CbCrTexID );
    glActiveTexture( GL_TEXTURE0 );
    glBindTexture( GL_TEXTURE_2D, YTexID );
    [self renderImage:flipped];
    checkGL( "render texture" );
    
    // set up perspective
    TooN::Vector<4> params = TooN::makeVector( 1179.90411, 1179.90411, 640, 360 );
    if ( flipped ) params[1] = -params[1];
    TooN::Matrix<4> proj = makeProj( params, 1280, 720 );
    
    TooN::Matrix<4> scale = TooN::Identity;
    scale(2,2) = -1;
    
    // set up camera pose
    TooN::Matrix<4> modelView = makeModelView( pose );
    
    // create model view proj
    TooN::Matrix<4> modelViewProj = proj * scale * modelView;
    
    if ( showPoints )
    {
        pointShader.shaderProgram.Use();
        pointShader.SetModelViewProj( modelViewProj );
        checkGL( "set up points" );
        
        // render points
        [self renderPoints];
        checkGL( "render points" );
    }
    
    if ( model && tracked ) {
        shadowShader.shaderProgram.Use();
        shadowShader.SetLightDirection( lightDirection );
        shadowShader.SetColor( shadowColor );
        shadowShader.SetPlane( plane );
        
        TooN::Matrix<4> modelScaleMat = makeScale( modelScale );
        TooN::Matrix<4> modelRotationMat = makeRotation( TooN::SO3<>( modelRotation ) );
        
        model->drawable->PushClientState();
        glEnable( GL_DEPTH_TEST );
        
        TooN::Matrix<4> modelTranslationMat;
        
        for ( int i = 0; i < modelOffsets.size(); i++ ) {
            modelTranslationMat = makeTranslation( modelOffsets[i] );
            modelViewProj = modelViewProj * modelTranslationMat * modelRotationMat * modelScaleMat;
            
            modelShader.shaderProgram.Use();
            modelShader.SetAmbient( 0.4 );
            modelShader.SetLightDirection( lightDirection );
            modelShader.SetModelViewProj( modelViewProj );
            model->Render();
            checkGL( "render model" );
            
            glActiveTexture( GL_TEXTURE1 );
            glBindTexture( GL_TEXTURE_2D, CbCrTexID );
            glActiveTexture( GL_TEXTURE0 );
            glBindTexture( GL_TEXTURE_2D, YTexID );

            glEnable( GL_POLYGON_OFFSET_FILL );
            glEnable( GL_BLEND );
            shadowShader.shaderProgram.Use();
            shadowShader.SetModelViewProj( modelViewProj );
            model->RenderNoTextureBind();
            glDisable( GL_BLEND );
            glDisable( GL_POLYGON_OFFSET_FILL );
            checkGL( "render shadow" );
        }
        
        model->drawable->PopClientState();
        glDisable( GL_DEPTH_TEST );
    }
}

- (void)renderWithPose:(TooN::SE3<>)pose
{
    if ( !ready ) return;
    
    // set context
    [EAGLContext setCurrentContext:context];

    // bind framebuffer
    glBindFramebuffer( GL_FRAMEBUFFER, framebuffer );
    
    // render to video if necessary
    if ( assetWriter && assetWriterInput.isReadyForMoreMediaData )
    {
        if ( videoTexture ) {
            CFRelease( videoTexture );
            videoTexture = NULL;
        }
        
        if ( videoBuffer ) {
            CVPixelBufferRelease( videoBuffer );
            videoBuffer = NULL;
        }
        
        CVOpenGLESTextureCacheFlush( textureCache, 0 );
        
        // see http://stackoverflow.com/questions/9550297/faster-alternative-to-glreadpixels-in-iphone-opengl-es-2-0
        
        CVPixelBufferPoolCreatePixelBuffer( NULL, assetWriterInputAdaptor.pixelBufferPool, &videoBuffer );
        
        // create texture linking CoreVideo and OpenGL
        CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault, textureCache, videoBuffer,
                                                     NULL, // texture attributes
                                                     GL_TEXTURE_2D,
                                                     GL_RGBA,
                                                     (int)backingWidth,
                                                     (int)backingHeight,
                                                     GL_BGRA, // native iOS format
                                                     GL_UNSIGNED_BYTE,
                                                     0,
                                                     &videoTexture);
        videoTexID =  CVOpenGLESTextureGetName(videoTexture);
        glBindTexture( GL_TEXTURE_2D, videoTexID );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, videoTexID, 0);
        
        glViewport( 0, 0, backingWidth, backingHeight );
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        
        [self _renderWithPose:pose flipped:NO];
        
        CVPixelBufferLockBaseAddress( videoBuffer, 0 );
        
        BOOL finished = [assetWriterInputAdaptor appendPixelBuffer:videoBuffer withPresentationTime:assetNextTime];
        if ( !finished ) [assetWriter finishWriting];
        
        assetNextTime.value++;
        
        CVPixelBufferUnlockBaseAddress( videoBuffer, 0 );
        
    }

    // set up framebuffer
    glBindRenderbuffer(GL_RENDERBUFFER, colorRenderbuffer);
    glFramebufferRenderbuffer( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorRenderbuffer );
    glViewport( 0, 0, backingWidth, backingHeight );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    checkGL( "bind framebuffer" );
    
    [self _renderWithPose:pose flipped:YES];
    
    // display renderbuffer
    glBindRenderbuffer(GL_RENDERBUFFER, colorRenderbuffer);
    [context presentRenderbuffer:GL_RENDERBUFFER];
}

- (void)startRecording:(NSURL *)url
{
    // set up video writer
    NSDictionary *outputSettings = [NSDictionary dictionaryWithObjectsAndKeys:
                                    AVVideoCodecH264,AVVideoCodecKey,
                                    [NSNumber numberWithInt:backingWidth],AVVideoWidthKey,
                                    [NSNumber numberWithInt:backingHeight],AVVideoHeightKey,
                                    nil];
    assetWriterInput = [[AVAssetWriterInput alloc] initWithMediaType:AVMediaTypeVideo outputSettings:outputSettings];
    assetWriterInput.expectsMediaDataInRealTime = YES;
    
    // create pixel buffer
    NSDictionary *pixbufAttr = [NSDictionary dictionaryWithObjectsAndKeys:
                                [NSNumber numberWithInt:kCVPixelFormatType_32BGRA],kCVPixelBufferPixelFormatTypeKey,
                                [NSNumber numberWithInt:backingWidth],kCVPixelBufferWidthKey,
                                [NSNumber numberWithInt:backingHeight],kCVPixelBufferHeightKey,
                                nil];
    assetWriterInputAdaptor = [[AVAssetWriterInputPixelBufferAdaptor alloc] initWithAssetWriterInput:assetWriterInput sourcePixelBufferAttributes:pixbufAttr];
    
    NSError *err;
    assetWriter = [[AVAssetWriter alloc] initWithURL:url fileType:AVFileTypeMPEG4 error:&err];
    [assetWriter addInput:assetWriterInput];

    [assetWriter startWriting];
    
    assetNextTime = CMTimeMake(0,15);
    [assetWriter startSessionAtSourceTime:assetNextTime];
}

- (void)stopRecording
{
    [assetWriter finishWriting];
    
    BOOL isCompatible = UIVideoAtPathIsCompatibleWithSavedPhotosAlbum( [assetWriter.outputURL path] );
    if ( isCompatible ) {
        UISaveVideoAtPathToSavedPhotosAlbum( [assetWriter.outputURL path], nil, nil, nil );
    }
        
    [assetWriter release];
    [assetWriterInput release];
    [assetWriterInputAdaptor release];
    
    assetWriter = nil;
}

@end
