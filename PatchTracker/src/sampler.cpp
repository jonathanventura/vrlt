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

    static inline bool in_image( const cv::Size &size, const cv::Point2f &loc )
    {
        return ( loc.x >= 0 && loc.x < size.width && loc.y >= 0 && loc.y < size.height );
    }

    static inline uchar getGraySubpix(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 1);
        
        int x = (int)pt.x;
        int y = (int)pt.y;
        
        int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
        int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
        int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
        int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);
        
        float a = pt.x - (float)x;
        float c = pt.y - (float)y;
        
        uchar b = (uchar)cvRound((img.at<uchar>(y0, x0) * (1.f - a) + img.at<uchar>(y0, x1) * a) * (1.f - c)
                                 + (img.at<uchar>(y1, x0) * (1.f - a) + img.at<uchar>(y1, x1) * a) * c);
        
        return b;
    }

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
        //cv::Mat M( 3, 3, CV_32FC1 );
        //cv::warpPerspective( sourceImage, templatePatch, M, templatePatch.size() );
        
        float offset = ( templatePatch.size().width - 1. ) / 2.;
        
        Eigen::Vector3f xstep = warp.col(0);
        Eigen::Vector3f ystep = warp.col(1);
        
        Eigen::Vector3f origin;
        origin << center[0] - offset, center[1] - offset, 1;
        origin = warp * origin;
        
        uchar *ptr = templatePatch.ptr();
        for ( int y = 0; y < 8; y++ )
        {
            Eigen::Vector3f X = origin + y*ystep;
            for ( int x = 0; x < 8; x++,X+=xstep,ptr++ )
            {
                Eigen::Vector2f pt = X.head(2)/X[2];
                pt[0] = ( pt[0] + .5f ) * scale - .5f;
                pt[1] = ( pt[1] + .5f ) * scale - .5f;
                cv::Point2f loc( pt[0], pt[1] );
                if ( !in_image( sourceImage.size(), loc ) ) return false;
                *ptr = getGraySubpix( sourceImage, loc );
            }
        }
        return true;
    }
}

