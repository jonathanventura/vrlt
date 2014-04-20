/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: outliers.h
 * Author: Jonathan Ventura
 * Last Modified: 20.04.2014
 */

#include <MultiView/multiview.h>

namespace vrlt
{
    int removeOutliers( Reconstruction *r, Node *node, double maxError = 16 );
    int removeOutliers( Reconstruction *r, Camera *camera, double maxError = 16 );
}
