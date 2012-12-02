/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: ssd.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/ssd.h>

#include <cvd/image_interpolate.h>
#include <cvd/image_convert.h>
#include <cvd/vision.h>

#include <iostream>

#ifdef USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#endif

#ifdef __ARM_NEON__
#include <arm_neon.h>
#endif

namespace vrlt {
    using namespace CVD;
    using namespace TooN;
    using namespace std;
    
    unsigned int getSSD( BasicImage<byte> &templatePatch, BasicImage<byte> &targetPatch )
    {
        unsigned int score = 0;
        
        byte *ptr1 = templatePatch.data();
        byte *ptr2 = targetPatch.data();
        
        for ( int i = 0; i < 64; i++,ptr1++,ptr2++ )
        {
            byte val1 = *ptr1;
            byte val2 = *ptr2;
            unsigned int diff = (val2>val1)?(val2-val1):(val1-val2);
            score += diff * diff;
        }
        return score;
    }
    
    void copyPatch( const BasicImage<byte> &from, const ImageRef &loc, BasicImage<byte> &to )
    {
        int x = loc.x;
        int y = loc.y;
        for ( int j = 0; j < 8; j++ )
        {
            const byte *ptr = from[y+j] + x;
            memcpy( to[j], ptr, 8 );
        }
    }
    
    bool samplePatch( BasicImage<byte> &sourceImage, const Vector<2> &center, BasicImage<float> &templatePatch )
    {
        image_interpolate<Interpolate::NearestNeighbour, byte> interpolate( sourceImage );

        double offset = ( templatePatch.size().x - 1. ) / 2.;
        
        ImageRef loc( 0, 0 );
        do {
            Vector<2> pt = makeVector( loc.x + center[0] - offset, loc.y + center[1] - offset );
            if ( !interpolate.in_image( pt ) ) return false;
            templatePatch[loc] = interpolate[pt];
        } while ( loc.next( templatePatch.size() ) );

        return true;
    }
    
    bool samplePatch( BasicImage<byte> &sourceImage, const Vector<2> &center, BasicImage<byte> &templatePatch )
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
    
    bool samplePatch( BasicImage<byte> &sourceImage, const Vector<2,float> &center, const Matrix<3,3,float> &warp, float scale, BasicImage<byte> &templatePatch )
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

    bool samplePatch( BasicImage<byte> &sourceImage, const Vector<2> &center, const Matrix<3> &warp, BasicImage<float> &templatePatch )
    {
        image_interpolate<Interpolate::Bilinear, byte> interpolate( sourceImage );

        double offset = ( templatePatch.size().x - 1. ) / 2.;
        
        ImageRef loc( 0, 0 );
        do {
            Vector<2> pt = project( warp * unproject( makeVector( loc.x + center[0] - offset, loc.y + center[1] - offset ) ) );
            if ( !interpolate.in_image( pt ) ) return false;
            templatePatch[loc] = interpolate[pt];
        } while ( loc.next( templatePatch.size() ) );
        
        return true;
    }
   
    void copyPatch( BasicImage<byte> &from, const ImageRef &loc, BasicImage<float> &to )
    {
#ifdef USE_ACCELERATE
        int x = loc.x;
        int y = loc.y;
        for ( int j = 0; j < 8; j++ )
        {
            byte *ptr = from[y+j] + x;
            vDSP_vfltu8( ptr, 1, to[j], 1, 8 );
        }
#else
        convert_image( from.sub_image( loc, to.size() ), to );
#endif
    }

    float getSSD( BasicImage<float> &templatePatch, BasicImage<float> &targetPatch, float matchThreshold )
    {
#ifdef USE_ACCELERATE
        float score = 0;
        int N = templatePatch.size().x * templatePatch.size().y;
        
        vDSP_vsub( templatePatch.data(), 1, targetPatch.data(), 1, targetPatch.data(), 1, N );
        vDSP_svesq( targetPatch.data(), 1, &score, N );
        
        return score;
#else
        float score = 0;
        
        ImageRef loc = ImageRef( 0, 0 );
        do {
            float templateVal = templatePatch[loc];
            float targetVal = targetPatch[loc];
            float diff = templateVal - targetVal;
            score += diff * diff;
            if ( score > matchThreshold ) break;
        } while ( loc.next( templatePatch.size() ) );
        
        return score;
#endif
    }
    
    SSDCalculator::SSDCalculator( ImageRef _sz ) : sz( _sz ), N( sz.x * sz.y ), tempData( sz )
    {
        
    }
    
    float SSDCalculator::getSSD( BasicImage<float> &templatePatch, BasicImage<float> &targetPatch, float matchThreshold )
    {
#ifdef USE_ACCELERATE
        float score = 0;
        
        vDSP_vsub( templatePatch.data(), 1, targetPatch.data(), 1, tempData.data(), 1, N );
        vDSP_svesq( tempData.data(), 1, &score, N );
        
        return score;
#else
        float score = 0;
        
        ImageRef loc = ImageRef( 0, 0 );
        do {
            float templateVal = templatePatch[loc];
            float targetVal = targetPatch[loc];
            float diff = templateVal - targetVal;
            score += diff * diff;
            if ( score > matchThreshold ) break;
        } while ( loc.next( templatePatch.size() ) );
        
        return score;
#endif
    }
    
    float getCorr( BasicImage<float> &im1, BasicImage<float> &im2 )
    {
#ifdef USE_ACCELERATE
        float corr;
        int N = im1.size().x * im1.size().y;
        vDSP_dotpr( im1.data(), 1, im2.data(), 1, &corr, N );
        return corr;
#else
        float corr = 0;
        int N = im1.size().x * im1.size().y;
        for ( int n = 0; n < N; n++ ) corr += im1.data()[n] * im2.data()[n];
        return corr;
#endif
    }
}
