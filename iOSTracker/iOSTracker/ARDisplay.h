/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: ARDisplay.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <OpenGLES/EAGL.h>

#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>

#include "GLSL.h"
#include "GLSLPrograms.h"
#include "GLModel.h"
#include "GLShadow.h"
#include "GLGenericDrawable.h"

#import <QuartzCore/CAEAGLLayer.h>

#import <AVFoundation/AVFoundation.h>

#import <CoreVideo/CVOpenGLESTextureCache.h>

#include "multiview.h"

@interface ARDisplay : NSObject
{
    BOOL ready;
    
    EAGLContext *context;
    
    GLuint framebuffer;
    GLuint colorRenderbuffer;
    GLuint depthRenderbuffer;
    
    CVOpenGLESTextureCacheRef textureCache;
    
    GLPointShader pointShader;
    GLGenericDrawable pointDrawable;
    
    GLYCbCrImageShader imageShader;
    GLGenericDrawable imageDrawable;
    GLGenericDrawable flippedImageDrawable;
    
    GLModelShader modelShader;
    GLShadowShader shadowShader;
    GLModel *model;

    GLuint numpoints;
    float *pointVertices;
    
    GLint backingWidth, backingHeight;
    
    std::vector< TooN::Vector<3> > modelOffsets;
    
    TooN::Vector<3> modelScale;
    TooN::Vector<3> modelTranslation;
    TooN::Vector<3> modelRotation;
    
    TooN::Vector<4> shadowColor;
    TooN::Vector<3> lightDirection;
    TooN::Vector<4> plane;
    
    vrlt::Node *node;
    
    BOOL showPoints;
    
    BOOL tracked;
    
    GLuint shadowFramebuffer;
    GLuint shadowTexture;
    GLImageShader shadowImageShader;
    GLGenericDrawable shadowImageDrawable;
    
    CVOpenGLESTextureRef YTexture;
    GLuint YTexID;
    
    CVOpenGLESTextureRef CbCrTexture;
    GLuint CbCrTexID;
    
    CVPixelBufferRef videoBuffer;
    
    CVOpenGLESTextureRef videoTexture;
    GLuint videoTexID;
    
    AVAssetWriter *assetWriter;
    AVAssetWriterInput *assetWriterInput;
    AVAssetWriterInputPixelBufferAdaptor *assetWriterInputAdaptor;
    CMTime assetNextTime;
}

- (id)initWithNode:(vrlt::Node*)_node;
- (BOOL)resizeFromLayer:(CAEAGLLayer *)layer;

- (void)setImageBuffer:(CVImageBufferRef)imageBuffer;

- (void)setOBJModelFromPath:(NSString*)path filename:(NSString*)filename;

- (void)addModelInstance;
- (void)setModelPlanePoint:(CGPoint)loc withPose:(TooN::SE3<>)pose andCalibration:(vrlt::Calibration*)calibration;

- (void)setModelScale:(NSString*)valString;
- (void)setModelTranslation:(NSString*)valString;
- (void)setModelRotation:(NSString*)valString;

- (void)setLightDirection:(NSString*)valString;
- (void)setPlane:(NSString*)valString;

- (void)renderWithPose:(TooN::SE3<>)pose;

- (void)startRecording:(NSURL*)url;
- (void)stopRecording;

@property (nonatomic,assign) BOOL showPoints;
@property (nonatomic,assign) BOOL tracked;

@end
