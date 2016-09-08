/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: features.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <FeatureExtraction/features.h>

#ifdef __linux__
#include <opencv2/imgproc/imgproc.hpp>
#else
#include <opencv2/imgproc.hpp>
#endif
#include <opencv2/xfeatures2d.hpp>

#include <iostream>

namespace vrlt {
    
    void normalizeFloats( float *floatdata )
    {
        float norm = 0;
        for ( int i = 0; i < 128; i++ ) norm += floatdata[i] * floatdata[i];
        norm = sqrt(norm);
        for ( int i = 0; i < 128; i++ ) floatdata[i] /= norm;
    }
    
    int extractSIFT( cv::Mat &image, std::vector<Feature*> &features, int o_min, double contrast_thresh )
    {
        cv::Mat gray_image;
        if ( image.channels() == 3 )
        {
            cv::cvtColor( image, gray_image, cv::COLOR_RGB2GRAY );
        }
        else
        {
            gray_image = image;
        }
        
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create( 0, 3, contrast_thresh );
        sift->detect( gray_image, keypoints );
        sift->compute( gray_image, keypoints, descriptors );
        
        float *floatdata = (float*)descriptors.ptr();
        
        features.clear();
        features.reserve( keypoints.size() );
        for ( size_t i = 0; i < keypoints.size(); i++,floatdata+=128 )
        {
            // according to this page:
            // https://groups.google.com/forum/#!topic/javacv/E9xngMMwVW4
            // SIFT octave is found by (keypoint.octave & 255)
            if ( (keypoints[i].octave & 255) < o_min ) continue;
            
            Feature *feature = new Feature;
            feature->location[0] = keypoints[i].pt.x;
            feature->location[1] = keypoints[i].pt.y;
            feature->scale = keypoints[i].size;
            feature->orientation = keypoints[i].angle;
            
            if ( image.channels() == 3 )
            {
                cv::Vec3b color = getColorSubpix( image, keypoints[i].pt );
                feature->color[0] = color[0];
                feature->color[1] = color[1];
                feature->color[2] = color[2];
            }
            else
            {
                uchar gray = getGraySubpix( image, keypoints[i].pt );
                feature->color[0] = gray;
                feature->color[1] = gray;
                feature->color[2] = gray;
            }
            
            normalizeFloats( floatdata );

            feature->descriptor = new unsigned char[128];
            for ( int k = 0; k < 128; k++ ) {
                float val = floatdata[k] * 512.f;
                if ( val > 255.f ) val = 255.f;
                feature->descriptor[k] = (unsigned char) val;
            }
            features.push_back( feature );
        }
        
        return features.size();
    }
}

