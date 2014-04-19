/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: search.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef SEARCH_H
#define SEARCH_H

#include <PatchTracker/patch.h>

namespace vrlt
{
    
/**
 * \addtogroup PatchTracker
 * @{
 */
    class PatchSearch
    {
    public:
        bool subsample;
        int level;
        float lowThreshold, highThreshold;
        std::vector<Patch*>::iterator begin;
        PatchSearch() : subsample( false ) { }
        virtual ~PatchSearch() { }
        
        virtual int makeTemplates( int count ) { return 0; }
        virtual int doSearch( int count ) { return 0; }
    };
    
/**
 * @}
 */
   
}

#endif
