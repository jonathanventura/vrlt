/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: featurematcher.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

#include <MultiView/multiview.h>

#include <FeatureMatcher/nn.h>

namespace vrlt {

/** \addtogroup FeatureMatcher
 * \brief Feature matching
 *
 * Helper functions for matching features between images.
 *
 * @{
 */
    
    /**
     * \brief Collect all features under a node into a list.
     *
     * \param[in] node          The root node of the tree to search.
     * \param[in] triangulated  Indicates whether only features of triangulated points should be included.
     * \param[out] features     The collected list of features under the node.
     */
    void addFeatures( Node *node, bool triangulated, std::vector<Feature*> &features );
    
    /**
     * \brief A generic feature matching class.  Builds an index of features to be matched against.
     */
    class FeatureMatcher {
    public:
        /**
         * \brief Constructor.
         *
         * \param[in] nn        The structure to be used for nearest neighbor calculations.
         * \param[in] _deleteNN Indicates whether the provided nearest neighbor structure should be deleted in the deconstructor of this class.  Defaults to false.
         */
        FeatureMatcher( NN *nn, bool _deleteNN = false );
        ~FeatureMatcher();
        
        /**
         * \brief Initialize of the feature matching index.
         *
         * \param[in] node                  The root node of the tree from which features will be added to the index.
         * \param[in] triangulated          Indicates whether only features of triangulated points should be included in the index.  Defaults to false.
         * \param[in] averageDescriptors    Indicates whether multiple feature descriptors from the same point should be averaged together.  Defaults to false.
         */
        void init( Node *node, bool triangulated = false, bool averageDescriptors = false );
        
        /**
         * \brief Find k nearest neighbors of a feature.
         *
         * \param[in] feature       The feature whose nearest neighbors will be found.
         * \param[in] k             The number of neighbors to return.
         * \param[out] neighbors    The indices of the neighbors. Must be pre-allocated.
         * \param[out] distances_sq The squared distances to the neighbors. Must be pre-allocated.
         */
        void search( Feature *feature, int k, int *neighbors, unsigned int *distances_sq );
        
        /**
         * \brief Find nearest neighbors of a list of features.
         *
         * \param[in] features      The features whose nearest neighbors will be found.
         * \param[out] neighbors    The indices of the neighbors. Must be pre-allocated.
         * \param[out] distances_sq The squared distances to the neighbors. Must be pre-allocated.
         */
        void search( std::vector<Feature *> &_features, int *neighbors, unsigned int *distances_sq );
        
        /**
         * \brief Find k nearest neighbors of a list of features.
         *
         * \param[in] features      The features whose nearest neighbors will be found.
         * \param[in] k             The number of neighbors per feature to return.
         * \param[out] neighbors    The indices of the neighbors, with the neighbors of the first feature listed first, then the neighbors of the second feature, and so on. Must be pre-allocated.
         * \param[out] distances_sq The squared distances to the neighbors. Must be pre-allocated.
         */
        void search( std::vector<Feature *> &_features, int k, int *neighbors, unsigned int *distances_sq );
        
        /**
         * \brief Find nearest neighbors of a list of features using the mutual consistency check.  Two features are only matched if they are mutually nearest neighbors.
         *
         * \param[in] features      The features whose mutually consistent nearest neighbors will be found.
         * \param[out] neighbors    The indices of the neighbors. Must be pre-allocated.
         * \param[out] distances_sq The squared distances to the neighbors. Must be pre-allocated.
         */
        void searchconsistent( std::vector<Feature *> &_features, int *neighbors, unsigned int *distances_sq );
        
        Feature * getfeature(int i) { return features[i]; };
        
        /**
         * \brief Returns whether the feature index is empty.
         */
        bool empty() { return features.empty(); }
    protected:
        std::vector<Feature*> features;
        unsigned char *data;
        NN *index;
		bool deleteNN;
        bool deleteFeatures;
    };
    
    /**
     * \brief Reverse matches so that the second feature is listed first.
     *
     * \param[in,out] matches      The matches to be reversed.
     */
    void reverseMatches( std::vector<Match*> &matches );
    
    /**
     * \brief Find nearest-neighbor matches for a set of features.
     *
     * \param[in] matcher       The feature matcher to be used for matching.
     * \param[in] features      The set of features to be matched.
     * \param[out] matches      The resulting feature matches.
     */
    void findMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, std::vector<Match*> &matches );

    /**
     * \brief Find k nearest-neighbor matches for a set of features.
     *
     * \param[in] matcher       The feature matcher to be used for matching.
     * \param[in] features      The set of features to be matched.
     * \param[in] k             The number of nearest neighbors per feature to be found.
     * \param[out] matches      The resulting feature matches.
     */
    void findMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, int k, std::vector<Match*> &matches );
    
    /**
     * \brief Find unique nearest-neighbor matches for a set of features by thresholding the ratio of the distance to the second nearest neighbor and the first nearest neighbor.
     *
     * \param[in] matcher       The feature matcher to be used for matching.
     * \param[in] features      The set of features to be matched.
     * \param[in] max_ratio     The maximum distance ratio to accept.
     * \param[out] matches      The resulting feature matches.
     */
    void findUniqueMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, double max_ratio, std::vector<Match*> &matches );
    
    /**
     * \brief Find mutually consistent nearest-neighbor matches for a set of features.
     *
     * \param[in] matcher       The feature matcher to be used for matching.
     * \param[in] features      The set of features to be matched.
     * \param[out] matches      The resulting feature matches.
     */
    void findConsistentMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, std::vector<Match*> &matches );

/**
 * @}
 */
}

#endif
