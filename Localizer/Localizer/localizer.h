/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: localizer.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <MultiView/multiview.h>
#include <PatchTracker/tracker.h>

namespace vrlt
{

/**
 * \addtogroup Localizer
 * \brief Image-based localization methods.
 * @{
 */
    class Localizer
    {
    public:
        Localizer( Node *_root, Node *_tracker_root = NULL );
        virtual ~Localizer();
        
        double thresh;
        double min_tracker_ratio;
        bool verbose;
        virtual bool localize( Camera *querycamera );
        
        bool refinePose( Camera *camera_in, float lambda );
        void refinePoseLM( Camera *camera_in, int niter );
        
    //protected:
        Node *root;
        Tracker *tracker;
    };    
/**
 * @}
 */
}

#endif
