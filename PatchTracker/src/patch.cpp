/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: patch.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/patch.h>
#include <PatchTracker/ncc.h>

#include <cvd/image.h>
#include <cvd/image_interpolate.h>
#include <cvd/image_convert.h>
#include <cvd/vision.h>

#include <TooN/svd.h>

#ifdef USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#endif

namespace vrlt
{
    using namespace std;
    using namespace TooN;
    using namespace CVD;
    
    Patch::Patch( Point *_point )
    : point( _point ), target( NULL ), source( NULL ), targetScore( INFINITY )
    {
        
    }
    
    static inline float scaleFromWarp( const TooN::Matrix<3,3,float> &warp )
    { 
        float det = warp(0,0) * warp(1,1) - warp(0,1) * warp(1,0);
        return sqrtf(det);
    }
    
    void Patch::setTarget( Camera *camera )
    {
        target = camera;
        
        SE3<> pose = target->node->pose;
        PX = pose * project( point->position );
        PN = pose.get_rotation() * point->normal;
        center = camera->calibration->project( project( PX ) );
        warpcenter = center;
        
        D = -(PN * PX);
        vTKinv[0] = camera->calibration->postMultiplyKinv( -(PN / D) );
        
        if ( source != NULL ) {
            calcWarp( source_feature );
            warp = tempWarp;
            scale = scaleFromWarp(warp);
        }
    }
    
    void Patch::updateTarget( Camera *camera )
    {
        target = camera;
        
        vTKinv[0] = camera->calibration->postMultiplyKinv( -(PN / D) );
        
        if ( source != NULL ) {
            calcWarp( source_feature );
            warp = tempWarp;
            scale = scaleFromWarp(warp);
        }
    }

    inline void Patch::calcWarp( Feature *feature )
    {
        Camera *camera = feature->camera;
#ifdef USE_ACCELERATE
        // make a * v.T() (the DSP call is really slow for some reason)
        //vDSP_mmul( camera->Ka.get_data_ptr(), 1, vTKinv.get_data_ptr(), 1, tempWarp.get_data_ptr(), 1, 3, 3, 1 );
        float *vTKinvptr = vTKinv.get_data_ptr();
        float *Kaptr = camera->Ka.get_data_ptr();
        float *tempWarpPtr = tempWarp.get_data_ptr();
        vDSP_vsmul( vTKinvptr, 1, Kaptr  , tempWarpPtr  , 1, 3 );
        vDSP_vsmul( vTKinvptr, 1, Kaptr+1, tempWarpPtr+3, 1, 3 );
        vDSP_vsmul( vTKinvptr, 1, Kaptr+2, tempWarpPtr+6, 1, 3 );
        // add K * A * target->Kinv
        vDSP_vadd( camera->KAKinv.get_data_ptr(), 1, tempWarpPtr, 1, tempWarpPtr, 1, 9 );
#else
        tempWarp = camera->KAKinv + camera->Ka * vTKinv;
#endif
    }
    
    bool Patch::chooseSource( bool use_new_cameras )
    {
        if ( target == NULL ) return false;
        
        source = NULL;
        warp = Identity;
        float bestDet = 0;
        
        ElementList::iterator it;
        for ( it = point->track->features.begin(); it != point->track->features.end(); it++ )
        {
            Feature *feature = (Feature *)it->second;
            Camera *camera = feature->camera;
            
            
            if ( !use_new_cameras && camera->isnew ) continue;
            
            calcWarp( feature );
            
            float det = scaleFromWarp( tempWarp );
            
            float mydet = det;
            if ( mydet > 1. ) mydet = 1. / mydet;
            
            bool accept = false;
            
            if ( source == NULL ) {
                accept = true;
            } else if ( camera->isnew && !source->isnew ) {
                accept = true;
            } else if ( !camera->isnew && source->isnew ) {
                accept = false;
            } else if ( det > bestDet ) {
                accept = true;
            }
            
            if ( accept ) {
                bestDet = mydet;
                warp = tempWarp;
                source = camera;
                source_feature = feature;
                scale = det;
            }
        }
        
        return ( source != NULL );
    }
}
