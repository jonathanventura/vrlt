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

#include <opencv2/highgui/highgui.hpp>

void loadJPEG_gray( const char *path, cv::Mat &image );

void loadJPEG_color( const char *path, cv::Mat &image );
