/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: pyramid.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef __PYRAMID_H
#define __PYRAMID_H

#include <cvd/image.h>
#include <cvd/byte.h>

#define NLEVELS 4

namespace vrlt {
    /** \addtogroup MultiView
     * @{
     */
    class Camera;
    
    /** \brief An image pyramid level
     *
     * One level of the image pyramid.
     */
    struct PyramidLevel {
        CVD::Image<CVD::byte> image;
    };
    
    /** \brief An image pyramid
     *
     * This class represents an image pyramid.
     * The levels of the pyramid are created by
     * repeated half-sampling.
     */
    struct ImagePyramid {
        ImagePyramid();
        ImagePyramid( CVD::ImageRef size );
        ImagePyramid( Camera *_camera );
        
        void resize( CVD::ImageRef size );
        void copy_from( const CVD::BasicImage<CVD::byte> &image_in );
        
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