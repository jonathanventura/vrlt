/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: nccsearch.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef NCC_SEARCH_H
#define NCC_SEARCH_H

#include <PatchTracker/search.h>

namespace vrlt
{
/**
 * \addtogroup PatchTracker
 * @{
 */

    class PatchSearchNCC : public PatchSearch
    {
    public:
        PatchSearchNCC( int maxnumpoints );
        ~PatchSearchNCC();
        
        virtual int makeTemplates( int count );
        virtual int doSearch( int count );
        
        CVD::byte *templates;
        CVD::byte *targets;
        float *templateA;
        float *templateC;
    };
    
/**
 * @}
 */
    
}

#endif
