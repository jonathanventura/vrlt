/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: LoadImage.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

void loadJPEG( const char *path, CVD::Image<CVD::byte> &image );

void loadJPEG( const char *path, CVD::Image< CVD::Rgb<CVD::byte> > &image );
