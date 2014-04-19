/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: patch.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef PATCH_H
#define PATCH_H

#include <MultiView/multiview.h>

#include <PatchTracker/sampler.h>

namespace vrlt
{

/**
 * \addtogroup PatchTracker
 * @{
 */
    class Patch
    {
    public:
        Sampler sampler;

        Patch( Point *_point );
        
        void setTarget( Camera *camera );

        bool copyTemplate( cv::Mat &output );
        
        void updateTarget( Camera *camera );
        bool chooseSource( bool use_new_cameras = true );
        inline void calcWarp( Feature *feature );
        
        Point *point;
        Camera *target;
        Camera *source;
        Feature *source_feature;
        bool shouldTrack;
        int bestLevel;
        size_t index;
        
        // best search location in target image
        Eigen::Vector2f targetPos;
        float targetScore;
        
        // from target to source
        Eigen::Matrix3f warp;
        Eigen::Vector2f warpcenter;
        float scale;
        
        // pre-computed values for speed-up
        Eigen::Vector3f PX;
        Eigen::Vector3f PN;
        float D;
        Eigen::Vector2f center;
        Eigen::RowVector3f vTKinv;
        Eigen::Matrix3f tempMat;
        Eigen::Matrix3f tempWarp;
        
        Eigen::Vector3f right;
        Eigen::Vector3f down;
    };
    
/**
 * @}
 */

}

#endif
