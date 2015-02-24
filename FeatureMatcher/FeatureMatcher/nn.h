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
    
    /**
     * \brief A generic nearest neighbor function class.
     */
    class NN
    {
    public:
        virtual ~NN() { }
        
        /**
         * \brief Set the descriptor data for features in the index.
         *
         * \param[in] _N    The number of descriptors provided.
         * \param[in] _data The descriptor data.  Must be of size 128*_N.
         */
        virtual void setData( int _N, unsigned char *_data ) { }
        
        /**
         * \brief Find consistent nearest neighbors for query features.
         *
         * \param[in] num_queries   The number of query descriptors provided.
         * \param[in] queries       The query descriptor data.  Must be of size 128*num_queries.
         * \param[out] neighbors    Indices of the nearest neighbors.  Must be pre-allocated.
         * \param[out] distances_sq Squared distances to the nearest neighbors.  Must be pre-allocated.
         */
        virtual void findconsistentnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq ) { }

        /**
         * \brief Find nearest neighbors for query features.
         *
         * \param[in] num_queries   The number of query descriptors provided.
         * \param[in] queries       The query descriptor data.  Must be of size 128*num_queries.
         * \param[out] neighbors    Indices of the nearest neighbors.  Must be pre-allocated.
         * \param[out] distances_sq Squared distances to the nearest neighbors.  Must be pre-allocated.
         */
        virtual void findnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq ) { }

        /**
         * \brief Find k nearest neighbors for query features.
         *
         * \param[in] num_queries   The number of query descriptors provided.
         * \param[in] queries       The query descriptor data.  Must be of size 128*num_queries.
         * \param[in] k             The number of neighbors per query feature to find.
         * \param[out] neighbors    Indices of the nearest neighbors.  The first indices are the k neighbors of the first query feature, and so on. Must be pre-allocated.
         * \param[out] distances_sq Squared distances to the nearest neighbors.  Must be pre-allocated.
         */
        virtual void findknn( int num_queries, unsigned char *queries, int k, int *neighbors, unsigned int *distances_sq ) { }
    };
    
    /**
     * @}
     */
}

#endif
