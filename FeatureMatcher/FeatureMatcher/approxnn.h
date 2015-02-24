/*
 * Copyright (c) 2014. Jonathan Ventura
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: approxnn.h
 * Author: Jonathan Ventura
 * Last Modified: 04.21.2014
 */

#ifndef BRUTE_FORCE_NN_H
#define BRUTE_FORCE_NN_H

#include "nn.h"

#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>

namespace vrlt {
/**
 * \addtogroup FeatureMatcher
 * @{
 */

    /**
     * \brief Approximate nearest neighbor implementation.  Wrapper for the OpenCV FLANN matcher.
     */
    class ApproxNN : public NN
    {
    public:
        int N;
        unsigned char *data;
        float *float_data;
        
        ApproxNN();
        ~ApproxNN();
        
        virtual void setData( int _N, unsigned char *_data );
        
        void findnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq );
        void findknn( int num_queries, unsigned char *queries, int k, int *neighbors, unsigned int *distances_sq );
    protected:
        cv::flann::GenericIndex< cv::flann::L2<unsigned char> > *flannindex;
    };

/**
 * @}
 */
}

#endif
