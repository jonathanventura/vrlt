/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: nnlocalizer.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef NN_LOCALIZER_H
#define NN_LOCALIZER_H

#include <Localizer/localizer.h>
#include <FeatureMatcher/featurematcher.h>

namespace vrlt
{

/**
 * \addtogroup Localizer
 * @{
 */
    class NNLocalizer : public Localizer
    {
    public:
        NNLocalizer( Node *_root, NN *index );
        ~NNLocalizer();
        
        bool localize( Camera *querycamera );
    protected:
        FeatureMatcher *fm;
        std::vector<Feature*> features;
        
        size_t N;
        
        friend void doFindMatches( void *context, size_t i );
    }; 
/**
 * @}
 */
}

#endif
