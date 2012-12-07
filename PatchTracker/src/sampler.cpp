/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: sampler.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/sampler.h>

#include <cstdio>

#include <cvd/vision.h>
#include <cvd/image_interpolate.h>

namespace vrlt {

    using namespace TooN;
    using namespace CVD;
    
    bool Sampler::samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2> &center, CVD::BasicImage<CVD::byte> &templatePatch )
    {
        double offset = ( templatePatch.size().x - 1. ) / 2.;
        Vector<2> origin = makeVector( center[0] - offset, center[1] - offset );
        Vector<2> bound = origin + makeVector(8,8);
        
        int w = sourceImage.size().x;
        int h = sourceImage.size().y;
        
        if ( origin[0] < 0 || origin[0] > w-1 ) return false;
        if ( origin[1] < 0 || origin[1] > h-1 ) return false;
        if ( bound[0] < 0 || bound[0] > w-1 ) return false;
        if ( bound[1] < 0 || bound[1] > h-1 ) return false;
        
        byte *ptr = templatePatch.data();
        for ( int y = 0; y < 8; y++ )
        {
            Vector<2> pt = makeVector( origin[0], origin[1] + y );
            for ( int x = 0; x < 8; x++,ptr++ )
            {
                sample( sourceImage, pt[0]+x, pt[1], (*ptr) );
            }
        }
        
        return true;
    }
    
    bool Sampler::samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2,float> &center, const TooN::Matrix<3,3,float> &warp, float scale, CVD::BasicImage<CVD::byte> &templatePatch )
    {
        image_interpolate<Interpolate::Bilinear, byte> interpolate( sourceImage );
        double offset = ( templatePatch.size().x - 1. ) / 2.;
        
        Vector<3> xstep = warp.T()[0];
        Vector<3> ystep = warp.T()[1];
        
        Vector<3> origin = warp * makeVector( center[0] - offset, center[1] - offset, 1. );
        
        byte *ptr = templatePatch.data();
        for ( int y = 0; y < 8; y++ )
        {
            Vector<3> X = origin + y*ystep;
            for ( int x = 0; x < 8; x++,X+=xstep,ptr++ )
            {
                Vector<2> pt = project( X );
                pt[0] = ( pt[0] + .5f ) * scale - .5f;
                pt[1] = ( pt[1] + .5f ) * scale - .5f;
                if ( !interpolate.in_image( pt ) ) return false;
                sample( sourceImage, pt[0], pt[1], (*ptr) );
            }
        }
        
        return true;
    }
}

