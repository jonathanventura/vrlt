/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
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

        bool getDepth( CVD::Image<float> &templateDepth );
        bool copyTemplate( CVD::Image<CVD::byte> &output );
        
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
        TooN::Vector<2,float> targetPos;
        float targetScore;
        
        // from target to source
        TooN::Matrix<3,3,float> warp;
        TooN::Vector<2,float> warpcenter;
        float scale;
        
        // pre-computed values for speed-up
        TooN::Vector<3,float> PX;
        TooN::Vector<3,float> PN;
        float D;
        TooN::Vector<2,float> center;
        TooN::Matrix<1,3,float> vTKinv;
        TooN::Matrix<3,3,float> tempMat;
        TooN::Matrix<3,3,float> tempWarp;
        
        TooN::Vector<3,float> right;
        TooN::Vector<3,float> down;
    };
    
/**
 * @}
 */

}

#endif
