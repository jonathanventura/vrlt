/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: ssd.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef SSD_H
#define SSD_H

#include <cvd/image.h>
#include <cvd/byte.h>
#include <TooN/TooN.h>

namespace vrlt {
    
    void adjustMinMax( CVD::BasicImage<CVD::byte> &patch );
    void copyPatch( const CVD::BasicImage<CVD::byte> &from, const CVD::ImageRef &loc, CVD::BasicImage<CVD::byte> &to );
    
    bool samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2> &center, CVD::BasicImage<float> &templatePatch );
    bool samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2> &center, CVD::BasicImage<CVD::byte> &templatePatch );
    bool samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2> &center, const TooN::Matrix<3> &warp, CVD::BasicImage<float> &templatePatch );
    bool samplePatch( CVD::BasicImage<CVD::byte> &sourceImage, const TooN::Vector<2,float> &center, const TooN::Matrix<3,3,float> &warp, float scale, CVD::BasicImage<CVD::byte> &templatePatch );
    void removeMean( CVD::BasicImage<float> &patch );
    void copyPatch( CVD::BasicImage<CVD::byte> &from, const CVD::ImageRef &loc, CVD::BasicImage<float> &to );
    float calcVariance( CVD::BasicImage<float> &zero_mean_patch );
    void normalize( CVD::BasicImage<float> &patch );
    
    class SSDCalculator
    {
    public:
        SSDCalculator( CVD::ImageRef _sz );
        float getSSD( CVD::BasicImage<float> &templatePatch, CVD::BasicImage<float> &targetPatch, float matchThreshold );
    private:
        CVD::ImageRef sz;
        int N;
        CVD::Image<float> tempData;
    };
    
    // warning: this is destructive to targetPatch
    float getSSD( CVD::BasicImage<float> &templatePatch, CVD::BasicImage<float> &targetPatch, float matchThreshold );

    unsigned int getSSD( CVD::BasicImage<CVD::byte> &templatePatch, CVD::BasicImage<CVD::byte> &targetPatch );
    
    float getCorr( CVD::BasicImage<float> &im1, CVD::BasicImage<float> &im2 );
}

#endif