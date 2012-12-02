/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: imagecache.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <MultiView/multiview.h>

#include <PatchTracker/ssd.h>

namespace vrlt
{  
    struct ImageCache
    {
        ImageCache();
        float matchThreshold;
        int maxCacheSize;
        Camera *bestCamera;
        
        void makeSmallByteImage( Camera *camera );
        void prepareSmallImage( Camera *camera );
        bool test( Camera *camera );
        void add( Camera *camera );
        void addCopy( Camera *camera );
        bool search( Camera *camera );
        
        void getNext( Camera *query );
        void moveNext();
        
        std::vector<Camera*> cache;
        int next_to_try;
        
        SSDCalculator ssd_calc;
    };
    
}
