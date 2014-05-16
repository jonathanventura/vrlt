/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: sampler.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef SAMPLER_H
#define SAMPLER_H

#include <Eigen/Core>
#include <opencv2/highgui.hpp>

namespace vrlt {
    
/**
 * \addtogroup PatchTracker
 * @{
 */

    struct Sampler
    {
        virtual bool samplePatch( cv::Mat &sourceImage, const Eigen::Vector2f &center, cv::Mat &templatePatch );
        virtual bool samplePatch( cv::Mat &sourceImage, const Eigen::Vector2f &center, const Eigen::Matrix3f &warp, float scale, cv::Mat &templatePatch );
    };

/**
 * @}
 */
}

#endif
