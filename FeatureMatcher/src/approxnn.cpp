/*
 * Copyright (c) 2014. Jonathan Ventura
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: approxnn.cpp
 * Author: Jonathan Ventura
 * Last Modified: 04.21.2014
 */

#include <FeatureMatcher/approxnn.h>

#include <cstdio>
#include <cstdlib>

namespace vrlt {
    
    ApproxNN::ApproxNN() : N(0), data(NULL), flannindex(NULL)
    {
        
    }
    
    void ApproxNN::setData( int _N, unsigned char *_data )
    {
        N = _N;
        data = _data;
        
        cv::Mat trainDescriptors( N, 128, CV_8UC1, data );
     
        ::cvflann::KDTreeIndexParams params;
        
        delete flannindex;
        flannindex = new cv::flann::GenericIndex< cv::flann::L2<unsigned char> >( trainDescriptors, params );
    }
    
    ApproxNN::~ApproxNN()
    {
        delete flannindex;
    }
    
    void ApproxNN::findnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq )
    {
        ::cvflann::SearchParams params;
        
        cv::Mat queryDescriptors( num_queries, 128, CV_8UC1, queries );
        cv::Mat indices( num_queries, 1, CV_32SC1, neighbors );
        cv::Mat dists( num_queries, 1, CV_32FC1 );
        flannindex->knnSearch( queryDescriptors, indices, dists, 1, params );
        
        unsigned int *distances_ptr = distances_sq;
        for ( int m = 0; m < num_queries; m++,distances_ptr++ )
        {
            *distances_ptr = (unsigned int)dists.at<float>(m);
        }
    }
    
    void ApproxNN::findknn( int num_queries, unsigned char *queries, int k, int *neighbors, unsigned int *distances_sq )
    {
        ::cvflann::SearchParams params;
        
        cv::Mat queryDescriptors( num_queries, 128, CV_8UC1, queries );
        cv::Mat indices( num_queries, k, CV_32SC1, neighbors );
        cv::Mat dists( num_queries, k, CV_32FC1 );
        flannindex->knnSearch( queryDescriptors, indices, dists, k, params );
        
        unsigned int *distances_ptr = distances_sq;
        for ( int m = 0; m < num_queries; m++ )
        {
            for ( int j = 0; j < k; j++,distances_ptr++ )
            {
                *distances_ptr = (unsigned int)dists.at<float>(m,j);
            }
        }
    }
    
}
