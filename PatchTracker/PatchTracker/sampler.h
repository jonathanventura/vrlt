/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: sampler.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef SAMPLER_H
#define SAMPLER_H

#include <TooN/TooN.h>
#include <cvd/image.h>
#include <cvd/byte.h>

namespace vrlt {
    
/**
 * \addtogroup PatchTracker
 * @{
 */

    struct Sampler
    {
        virtual bool samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2> &center, CVD::BasicImage<CVD::byte> &templatePatch );
        virtual bool samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2,float> &center, const TooN::Matrix<3,3,float> &warp, float scale, CVD::BasicImage<CVD::byte> &templatePatch );
    };

/**
 * @}
 */
}

#endif
