/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: nn.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef NN_H
#define NN_H

namespace vrlt {
/**
 * \addtogroup FeatureMatcher
 * @{
 */

class NN
{
public:
    virtual void setData( int _N, unsigned char *_data ) { }
    
    virtual void getDistances( int num_queries, unsigned char *queries, unsigned int *stored_distsqs ) { }
    
    virtual void findconsistentnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq ) { }
    
    virtual void findnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq ) { }
    virtual void findknn( int num_queries, unsigned char *queries, int k, int *neighbors, unsigned int *distances_sq ) { }
};

/**
 * @}
 */
}

#endif
