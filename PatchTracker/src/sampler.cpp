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

#include <Eigen/Core>

#include <opencv2/imgproc.hpp>

namespace vrlt {

    bool Sampler::samplePatch( cv::Mat &sourceImage, const Eigen::Vector2d &center, cv::Mat &templatePatch )
    {
        cv::getRectSubPix( sourceImage, templatePatch.size(), cv::Point2f( center[0], center[1] ), templatePatch );
        /*
        double offset = ( templatePatch.size().width - 1. ) / 2.;
        Eigen::Vector2f origin;
        origin << center[0] - offset, center[1] - offset;
        Eigen::Vector2f bound;
        bound << origin[0]+8, origin[1]+8;
        
        int w = sourceImage.size().width;
        int h = sourceImage.size().height;
        
        if ( origin[0] < 0 || origin[0] > w-1 ) return false;
        if ( origin[1] < 0 || origin[1] > h-1 ) return false;
        if ( bound[0] < 0 || bound[0] > w-1 ) return false;
        if ( bound[1] < 0 || bound[1] > h-1 ) return false;
        
        uchar *ptr = templatePatch.ptr();
        for ( int y = 0; y < 8; y++ )
        {
            Eigen::Vector2f pt = makeVector( origin[0], origin[1] + y );
            for ( int x = 0; x < 8; x++,ptr++ )
            {
                sample( sourceImage, pt[0]+x, pt[1], (*ptr) );
            }
        }
        */
        return true;
    }
    
    bool Sampler::samplePatch( cv::Mat &sourceImage, const Eigen::Vector2f &center, const Eigen::Matrix3f &warp, float scale, cv::Mat &templatePatch )
    {
        cv::Mat M( 3, 3, CV_32FC1 );
        // need to figure out M here
        cv::warpPerspective( sourceImage, templatePatch, M, templatePatch.size() );
        /*
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
        */
        return true;
    }
}

