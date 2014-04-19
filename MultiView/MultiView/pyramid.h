/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: pyramid.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef __PYRAMID_H
#define __PYRAMID_H

#include <opencv2/highgui.hpp>

#define NLEVELS 4

namespace vrlt {
    /** \addtogroup MultiView
     * @{
     */
    struct Camera;
    
    /** \brief An image pyramid level
     *
     * One level of the image pyramid.
     */
    struct PyramidLevel {
        cv::Mat image;
    };
    
    /** \brief An image pyramid
     *
     * This class represents an image pyramid.
     * The levels of the pyramid are created by
     * repeated half-sampling.
     */
    struct ImagePyramid {
        ImagePyramid();
        ImagePyramid( cv::Size size );
        ImagePyramid( Camera *_camera );
        
        void resize( cv::Size size );
        void copy_from( const cv::Mat &image_in );
        
        /** Remake the pyramid level images.  Useful when the camera image changes, to avoid re-allocating space for the pyramid levels. */
        void remake();
        
        Camera *camera;
        PyramidLevel levels[NLEVELS];
    };
    
    /**
     * @}
     */
}

#endif