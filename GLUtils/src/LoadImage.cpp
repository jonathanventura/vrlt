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

static void copyImageData_gray( CGImageRef imageRef, cv::Mat &image )
{
    size_t width = CGImageGetWidth(imageRef);
    size_t height = CGImageGetHeight(imageRef);
    image = cv::Mat( cv::Size( width, height ), CV_8UC1 );
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceGray();
    unsigned char *rawData = image.data;
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

void loadJPEG_gray( const char *path, cv::Mat &image )
{
    CGDataProviderRef jpegDataProvider = CGDataProviderCreateWithFilename( path );
    
    CGImageRef imageRef = CGImageCreateWithJPEGDataProvider( jpegDataProvider, NULL, false, kCGRenderingIntentDefault );
    
    copyImageData_gray( imageRef, image );
    
    CGImageRelease( imageRef );
    CGDataProviderRelease( jpegDataProvider );
}

static void copyImageData_color( CGImageRef imageRef, cv::Mat &image )
{
    size_t width = CGImageGetWidth(imageRef);
    size_t height = CGImageGetHeight(imageRef);
    image = cv::Mat( cv::Size( width, height ), CV_8UC3 );
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    unsigned char *rawData = image.data;
    size_t bytesPerPixel = 4;
    size_t bytesPerRow = bytesPerPixel * width;
    size_t bitsPerComponent = 8;
    CGContextRef context = CGBitmapContextCreate(rawData, width, height,
                                                 bitsPerComponent, bytesPerRow, colorSpace,
                                                 kCGImageAlphaPremultipliedLast | kCGBitmapByteOrderDefault);
    CGColorSpaceRelease(colorSpace);
    
    CGContextDrawImage(context, CGRectMake(0, 0, width, height), imageRef);
    CGContextRelease(context);
}

void loadJPEG_color( const char *path, cv::Mat &image )
{
    CGDataProviderRef jpegDataProvider = CGDataProviderCreateWithFilename( path );
    
    CGImageRef imageRef = CGImageCreateWithJPEGDataProvider( jpegDataProvider, NULL, false, kCGRenderingIntentDefault );
    
    copyImageData_color( imageRef, image );
    
    CGImageRelease( imageRef );
    CGDataProviderRelease( jpegDataProvider );
}

