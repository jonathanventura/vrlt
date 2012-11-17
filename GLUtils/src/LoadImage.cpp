/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: LoadImage.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <GLUtils/LoadImage.h>

#include <CoreGraphics/CoreGraphics.h>

static void copyImageData( CGImageRef imageRef, CVD::Image<CVD::byte> &image )
{
    size_t width = CGImageGetWidth(imageRef);
    size_t height = CGImageGetHeight(imageRef);
    image.resize( CVD::ImageRef( width, height ) );
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceGray();
    unsigned char *rawData = image.data();
    size_t bytesPerPixel = 1;
    size_t bytesPerRow = bytesPerPixel * width;
    size_t bitsPerComponent = 8;
    CGContextRef context = CGBitmapContextCreate(rawData, width, height,
                                                 bitsPerComponent, bytesPerRow, colorSpace,
                                                 kCGImageAlphaNone | kCGBitmapByteOrderDefault);
    CGColorSpaceRelease(colorSpace);
    
    CGContextDrawImage(context, CGRectMake(0, 0, width, height), imageRef);
    CGContextRelease(context);
}

void loadJPEG( const char *path, CVD::Image<CVD::byte> &image )
{
    CGDataProviderRef jpegDataProvider = CGDataProviderCreateWithFilename( path );
    
    CGImageRef imageRef = CGImageCreateWithJPEGDataProvider( jpegDataProvider, NULL, false, kCGRenderingIntentDefault );
    
    copyImageData( imageRef, image );
    
    CGImageRelease( imageRef );
    CGDataProviderRelease( jpegDataProvider );
}

static void copyImageData( CGImageRef imageRef, CVD::Image< CVD::Rgb<CVD::byte> > &image )
{
    size_t width = CGImageGetWidth(imageRef);
    size_t height = CGImageGetHeight(imageRef);
    image.resize( CVD::ImageRef( width, height ) );
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    unsigned char *rawData = (unsigned char *)malloc( width * height * 4 );
    size_t bytesPerPixel = 4;
    size_t bytesPerRow = bytesPerPixel * width;
    size_t bitsPerComponent = 8;
    CGContextRef context = CGBitmapContextCreate(rawData, width, height,
                                                 bitsPerComponent, bytesPerRow, colorSpace,
                                                 kCGImageAlphaPremultipliedLast | kCGBitmapByteOrderDefault);
    CGColorSpaceRelease(colorSpace);
    
    CGContextDrawImage(context, CGRectMake(0, 0, width, height), imageRef);
    CGContextRelease(context);
    
    for ( int i = 0; i < width*height; i++ ) {
        image.data()[i].red =   rawData[i*4+0];
        image.data()[i].green = rawData[i*4+1];
        image.data()[i].blue =  rawData[i*4+2];
    }
    
    free( rawData );
}

void loadJPEG( const char *path, CVD::Image< CVD::Rgb<CVD::byte> > &image )
{
    CGDataProviderRef jpegDataProvider = CGDataProviderCreateWithFilename( path );
    
    CGImageRef imageRef = CGImageCreateWithJPEGDataProvider( jpegDataProvider, NULL, false, kCGRenderingIntentDefault );
    
    copyImageData( imageRef, image );
    
    CGImageRelease( imageRef );
    CGDataProviderRelease( jpegDataProvider );
}

