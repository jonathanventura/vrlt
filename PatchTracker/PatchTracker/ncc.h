/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: ncc.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

namespace vrlt {
    
/**
 * \addtogroup PatchTracker
 * @{
 */
    void memcpy8( unsigned char *output, unsigned char *input );

    // assuming we have 8x8 patch = 64 elements here
    unsigned int computeSum( unsigned char *x );
    unsigned int computeSumSq( unsigned char *x );
    unsigned int computeDotProduct( unsigned char *x, unsigned char *y );

    unsigned int computeSum( unsigned char *x, int rowstep );
    unsigned int computeSumSq( unsigned char *x, int rowstep );
    unsigned int computeDotProduct( unsigned char *x, int rowstep, unsigned char *y );

/**
 * @}
 */

}