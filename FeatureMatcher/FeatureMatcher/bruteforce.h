/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: bruteforce.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef BRUTE_FORCE_NN_H
#define BRUTE_FORCE_NN_H

#include "NN.h"

#include <OpenCL/opencl.h>

/**
 * \addtogroup FeatureMatcher
 * @{
 */

class BruteForceNN : public NN
{
public:
    int N;
    unsigned char *data;
    float *float_data;
    
    cl_device_id device_id;
	cl_context context;
    
    cl_mem data_mem;
    
    int response_buf_size;
    cl_mem response_buf_mem;
    
    cl_program dists_program;
    
    void compileProgram( cl_program prog );
    
    BruteForceNN();
    ~BruteForceNN();
    
    virtual void setData( int _N, unsigned char *_data );
    
    void findconsistentnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq );
    
    void findnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq );
    void findknn( int num_queries, unsigned char *queries, int k, int *neighbors, unsigned int *distances_sq );
    
    virtual void getDistances( size_t num_queries, unsigned char *queries, unsigned int *stored_distsqs );
};

/**
 * @}
 */

#endif
