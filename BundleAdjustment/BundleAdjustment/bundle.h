/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: bundle.h
 * Author: Jonathan Ventura
 * Last Modified: 20.04.2014
 */
#include <MultiView/multiview.h>

namespace vrlt
{
    class BundleInternal;
    
    class Bundle
    {
    public:
        Bundle( Node *_root, bool _verbose = false );
        Bundle( Node *_root, const ElementList &_fixedNodes, const ElementList &_fixedPoints, bool _verbose = false );
        ~Bundle();
        
        bool run();
        void run_str();
        void run_mot();
    protected:
        BundleInternal *internal;
    };
    
    void fixScale( Node *root );
    bool runBundle( Reconstruction *r );
}
