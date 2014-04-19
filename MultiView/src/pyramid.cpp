/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: pyramid.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <MultiView/multiview.h>

#include <opencv2/imgproc.hpp>

namespace vrlt {
       
    ImagePyramid::ImagePyramid() : camera( NULL )
    {
        
    }
    
    ImagePyramid::ImagePyramid( cv::Size size )
    {
        this->resize( size );
    }
    
    ImagePyramid::ImagePyramid( Camera *_camera ) : camera( _camera )
    {
        levels[0].image = camera->image;
        for ( int i = 1; i < NLEVELS; i++ ) {
            cv::pyrDown( levels[i-1].image, levels[i].image );
        }
    }
    
    void ImagePyramid::resize( cv::Size size )
    {
        cv::Size mysize = size;
        levels[0].image.create( mysize, CV_8UC1 );
        for ( int i = 1; i < NLEVELS; i++ ) {
            mysize.width /= 2;
            mysize.height /= 2;
            levels[i].image.create( mysize, CV_8UC1 );
        }
    }
    
    void ImagePyramid::copy_from( const cv::Mat &image_in )
    {
        image_in.copyTo( levels[0].image );
        remake();
    }
    
    void ImagePyramid::remake()
    {
        for ( int i = 1; i < NLEVELS; i++ ) {
            cv::pyrDown( levels[i-1].image, levels[i].image );
        }
    }
    
}
