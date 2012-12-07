/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: robustlsq.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef ROBUST_LEASTSQ
#define ROBUST_LEASTSQ

#include <MultiView/multiview.h>
#include <PatchTracker/patch.h>
#include <vector>

namespace vrlt {

    struct RobustLeastSq
    { 
        int niter;
        float ksq;
        Node *root;
    
        RobustLeastSq( Node *_root ) : root( _root ), niter( 10 ), ksq( 1.f )
        {
        }

        bool updatePose( Camera *camera_in, int count, int iter, float eps );

        bool run( Camera *camera_in, int count );
    };
}

#endif

