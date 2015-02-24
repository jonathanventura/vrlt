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
 *
 * Helper functions for detecting feature points, extracting feature descriptors and loading them into the Feature structure.
 *
 * @{
 *
 */
    /**
     * \brief Extract SIFT features from an image.
     *
     * \param[in] image               The image from which features will be extracted
     * \param[out] features           Vector of feature structures
     * \param[in] o_min               Minimum octave at which features will be extracted, defaults to 0
     * \param[in] contrast_thresh     Contrast threshold for SIFT feature detector, defaults to 0.04
     */
    int extractSIFT( cv::Mat &image, std::vector<Feature*> &features, int o_min = 0, double contrast_thresh = 0.04 );
/**
 * @}
 */
}

#endif
