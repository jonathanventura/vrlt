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

#include <opencv2/core/eigen.hpp>

namespace vrlt {

    bool Sampler::samplePatch( cv::Mat &sourceImage, const Eigen::Vector2d &center, cv::Mat &templatePatch )
    {
        cv::getRectSubPix( sourceImage, templatePatch.size(), cv::Point2f( center[0], center[1] ), templatePatch );
        return true;
    }
    
    bool Sampler::samplePatch( cv::Mat &sourceImage, const Eigen::Vector2f &center, const Eigen::Matrix3f &warp, float scale, cv::Mat &templatePatch )
    {
        float offset = ( templatePatch.size().width - 1. ) / 2.;
        
        Eigen::Matrix3f M;
        M <<
        1, 0, center[0]-offset,
        0, 1, center[1]-offset,
        0, 0, 1;
        
        M = warp * M;
        
        Eigen::Matrix3f scale_mat;
        scale_mat <<
        scale, 0, 0.5f*scale-0.5f,
        0, scale, 0.5f*scale-0.5f,
        0, 0, 1;
        
        M = scale_mat * M;
        
        cv::Mat cvM( 3, 3, CV_32FC1 );
        cv::eigen2cv( M, cvM );
        cv::warpPerspective(sourceImage, templatePatch, cvM, templatePatch.size(), cv::WARP_INVERSE_MAP );
        
        return true;
    }
}

