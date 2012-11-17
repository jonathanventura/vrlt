/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: featurematcher.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

#include <MultiView/multiview.h>

#include <FeatureMatcher/nn.h>

namespace MultiView {

/** \addtogroup FeatureMatcher
 * \brief Feature matching
 * @{
 */
    
    void addFeatures( Node *node, bool triangulated, std::vector<Feature*> &features );
    
    class FeatureMatcher {
    public:
        FeatureMatcher( NN *nn, bool _deleteNN = false );
        ~FeatureMatcher();
        
        void init( Node *node, bool triangulated = false, bool averageDescriptors = false );
        
        void search( Feature *feature, int k, int *neighbors, unsigned int *distances_sq );
        void search( std::vector<Feature *> &_features, int *neighbors, unsigned int *distances_sq );
        void search( std::vector<Feature *> &_features, int k, int *neighbors, unsigned int *distances_sq );
        
        void searchconsistent( std::vector<Feature *> &_features, int *neighbors, unsigned int *distances_sq );
        
        std::vector<Feature*> features;
    protected:
        unsigned char *data;
        NN *index;
		bool deleteNN;
        bool deleteFeatures;
    };
    
    void reverseMatches( std::vector<Match*> &matches );
    void findMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, std::vector<Match*> &matches );
    void findMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, int k, std::vector<Match*> &matches );
    void findUniqueMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, double max_ratio, std::vector<Match*> &matches );
    void findConsistentMatches( FeatureMatcher &matcher, std::vector<Feature*> &features, std::vector<Match*> &matches );

/**
 * @}
 */
}

#endif
