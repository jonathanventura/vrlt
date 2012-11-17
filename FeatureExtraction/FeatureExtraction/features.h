/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: features.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef __FEATURES_H
#define __FEATURES_H

#include <MultiView/multiview.h>
#include <cvd/rgb.h>

namespace vrlt {

/** \addtogroup FeatureExtraction
 * \brief Feature extraction
 * @{
 */
    int detectShiTomasi( CVD::BasicImage<CVD::byte> &image, std::vector<Feature*> &features, int numcorners );
    int detectHarris( CVD::BasicImage<CVD::byte> &image, std::vector<Feature*> &features, int numcorners );
    
    int detectFAST( CVD::BasicImage<CVD::byte> &image, std::vector<Feature*> &features, int thresh = 10 );
    int detectFAST( ImagePyramid &image, std::vector<Feature*> &features );
    
    int extractPatchDescriptors( CVD::BasicImage< CVD::Rgb<CVD::byte> > &color_image, std::vector<Feature*> &features, std::vector<Feature*> &features_out );
    
    int extractSIFTdescriptors( CVD::BasicImage< CVD::byte > &image, std::vector<Feature*> &features, std::vector<Feature*> &features_out );
    int extractSIFTdescriptors( CVD::BasicImage< CVD::Rgb<CVD::byte> > &color_image, std::vector<Feature*> &features, std::vector<Feature*> &features_out );
    int extractSIFT( CVD::BasicImage<CVD::byte> &image, std::vector<Feature*> &features, int o_min = 0, bool upright = false, float peak_thresh = 0.5f );
    int extractSIFT( CVD::BasicImage< CVD::Rgb<CVD::byte> > &color_image, std::vector<Feature*> &features, int o_min = 0, bool upright = false, float peak_thresh = 0.5f );
/**
 * @}
 */
}

#endif
