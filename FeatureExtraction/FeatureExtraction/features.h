/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: features.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef __FEATURES_H
#define __FEATURES_H

#include <MultiView/multiview.h>

namespace vrlt {

/** \addtogroup FeatureExtraction
 * \brief Feature extraction
 * @{
 */
//    int detectShiTomasi( cv::Mat &image, std::vector<Feature*> &features, int numcorners );
//    int detectHarris( cv::Mat &image, std::vector<Feature*> &features, int numcorners );
    
//    int detectFAST( cv::Mat &image, std::vector<Feature*> &features, int thresh = 10 );
//    int detectFAST( ImagePyramid &image, std::vector<Feature*> &features );
    
//    int extractPatchDescriptors( cv::Mat &color_image, std::vector<Feature*> &features, std::vector<Feature*> &features_out );
    
//    int extractSIFTdescriptors( cv::Mat &image, std::vector<Feature*> &features, std::vector<Feature*> &features_out );
    int extractSIFT( cv::Mat &image, std::vector<Feature*> &features, int o_min = 0, bool upright = false, float peak_thresh = 0.5f );
/**
 * @}
 */
}

#endif
