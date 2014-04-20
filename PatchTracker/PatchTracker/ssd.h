/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: ssd.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef SSD_H
#define SSD_H

#include <opencv2/highgui.hpp>
#include <Eigen/Core>

namespace vrlt {
    //void adjustMinMax( cv::Mat &patch );
    //void copyPatch( const cv::Mat &from, const cv::Point2i &loc, cv::Mat &to );
    
    //bool samplePatch( cv::Mat &sourceImage, const Eigen::Vector2d &center, cv::Mat &templatePatch );
    //bool samplePatch( cv::Mat &sourceImage, const Eigen::Vector2d &center, cv::Mat &templatePatch );
    //bool samplePatch( cv::Mat &sourceImage, const Eigen::Vector2d &center, const Eigen::Matrix3d &warp, cv::Mat &templatePatch );
    //bool samplePatch( cv::Mat &sourceImage, const Eigen::Vector2f &center, const Eigen::Matrix3f &warp, float scale, cv::Mat &templatePatch );
    
    //void removeMean( cv::Mat &patch );
    //void copyPatch( cv::Mat &from, const cv::Point2i &loc, cv::Mat &to );
    //float calcVariance( cv::Mat &zero_mean_patch );
    //void normalize( cv::Mat &patch );
    
    class SSDCalculator
    {
    public:
        SSDCalculator( cv::Size _sz );
        float getSSD( cv::Mat &templatePatch, cv::Mat &targetPatch, float matchThreshold );
    private:
        cv::Size sz;
        int N;
        cv::Mat tempData;
    };
    /*
    // warning: this is destructive to targetPatch
    float getSSD( cv::Mat &templatePatch, cv::Mat &targetPatch, float matchThreshold );

    unsigned int getSSD( cv::Mat &templatePatch, cv::Mat &targetPatch );
    */
    float getCorr( cv::Mat &im1, cv::Mat &im2 );
}

#endif