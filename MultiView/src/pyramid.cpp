/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: pyramid.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <MultiView/multiview.h>

#include <cvd/image_io.h>
#include <cvd/vision.h>

namespace vrlt {
    
    using namespace CVD;
    
    ImagePyramid::ImagePyramid() : camera( NULL )
    {
        
    }
    
    ImagePyramid::ImagePyramid( ImageRef size )
    {
        this->resize( size );
    }
    
    ImagePyramid::ImagePyramid( Camera *_camera ) : camera( _camera )
    {   
        levels[0].image = camera->image;
        for ( int i = 1; i < NLEVELS; i++ ) {
            levels[i].image = halfSample( levels[i-1].image );
        }
    }
    
    void ImagePyramid::resize( CVD::ImageRef size )
    {
        levels[0].image.resize( size );
        for ( int i = 1; i < NLEVELS; i++ ) {
            levels[i].image.resize( levels[i-1].image.size() / 2 );
        }
    }
    
    void ImagePyramid::copy_from( const CVD::BasicImage<CVD::byte> &image_in )
    {
        levels[0].image.copy_from( image_in );
        remake();
    }
    
    void ImagePyramid::remake()
    {
        for ( int i = 1; i < NLEVELS; i++ ) {
            halfSample( levels[i-1].image, levels[i].image );
        }
    }
    
}
