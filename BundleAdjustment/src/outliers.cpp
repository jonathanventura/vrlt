/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: outliers.cpp
 * Author: Jonathan Ventura
 * Last Modified: 20.04.2014
 */

#include <BundleAdjustment/outliers.h>

#include <Estimator/estimator.h>

#include <iostream>

namespace vrlt
{
    int removeFarAwayPoints( Reconstruction *r, Node *node )
    {
        std::vector<Point*> pointsToRemove;
        
        ElementList::iterator it;
        for ( it = node->points.begin(); it != node->points.end(); it++ )
        {
            Point *point = (Point*)it->second;
            Eigen::Vector4d pt = point->position;
            if ( fabs( pt[3] ) / pt.head(3).norm() < .05 ) {
                pointsToRemove.push_back( point );
            }
        }
        
        for ( int i = 0; i < pointsToRemove.size(); i++ )
        {
            Point *point = pointsToRemove[i];
            point->track->point = NULL;
            node->points.erase( point->name );
            delete point;
        }
        
        return (int)pointsToRemove.size();
    }

    int removeOutliers( Reconstruction *r, Node *node, double maxError )
    {
        int numremoved = 0;
        
        //numremoved += removeFarAwayPoints( r, node );
        
        if ( node->camera != NULL ) {
            numremoved += removeOutliers( r, node->camera, maxError );
        }
        
        ElementList::iterator it;
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node*)it->second;
            numremoved += removeOutliers( r, child, maxError );
        }
        
        return numremoved;
    }

    int removeOutliers( Reconstruction *r, Camera *camera, double maxError )
    {
        // collect all reprojection errors
        std::vector<double> errors;
        std::vector<Feature*> features;
        
        double sum = 0;
        
        ElementList::iterator it;
        for ( it = camera->features.begin(); it != camera->features.end(); it++ )
        {
            Feature *feature = (Feature *)it->second;
            if ( feature->track == NULL ) continue;
            if ( feature->track->point == NULL ) continue;
            
            Eigen::Vector2d diff = computeReprojError( feature );
            double err = diff.dot(diff);
            sum += err;
            
            errors.push_back( err );
            features.push_back( feature );
        }
        
        if ( features.empty() ) return 0;
        
        // find 80-th percentile
        std::vector<double> sortedErrors = errors;
        std::sort( sortedErrors.begin(), sortedErrors.end() );

        int percentile = (int)errors.size() * 8 / 10;
        double threshold = 2.4 * sortedErrors[percentile];
        if ( threshold < 4 ) threshold = 4;
        if ( threshold > maxError ) threshold = maxError;
        
        int numRemoved = 0;
        for ( int i = 0; i < features.size(); i++ )
        {
            Track *track = features[i]->track;
            if ( track == NULL ) continue;
            
            if ( errors[i] > threshold )
            {
                numRemoved++;
                r->remove( track );
                continue;
            }
        }
        
        return numRemoved;
    }
}
