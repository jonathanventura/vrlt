/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: localizer.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <Localizer/localizer.h>
#include <PatchTracker/tracker.h>

#include <TooN/Lapack_Cholesky.h>

using namespace TooN;

namespace vrlt
{
    Localizer::Localizer( Node *_root, Node *_tracker_root )
    : root( _root ), verbose( false )
    {
        if ( _tracker_root == NULL ) _tracker_root = root->root();
        tracker = new Tracker( _tracker_root, 1024 );
        tracker->niter = 10;
        tracker->firstlevel = 1;
        tracker->lastlevel = 0;
        tracker->minnumpoints = 100;
        min_tracker_ratio = 0.1;
        tracker->minratio = min_tracker_ratio;
    }
    
    Localizer::~Localizer()
    {
        delete tracker;
    }
    
    bool Localizer::localize( Camera *querycamera )
    {
        return false;
        
    }

    bool Localizer::refinePose( Camera *camera_in, float lambda )
    {
        Matrix<6> FtF = Zeros;
        Vector<6> Fte = Zeros;
        
        SE3<> pose( camera_in->node->pose );
        float f = camera_in->calibration->focal;
        Vector<2,float> center = camera_in->calibration->center;
        
        float toterr = 0.f;
        
        ElementList::iterator it;
        for ( it = camera_in->features.begin(); it != camera_in->features.end(); it++ )
        {
            Feature *feature = (Feature*)it->second;
            
            Track *track = feature->track;
            if ( track == NULL ) continue;
            Point *point = track->point;
            if ( point == NULL ) continue;
            
            Vector<3,float> PX = pose * project( point->position );
            
            Matrix<2,3,float> A;
            A[0] = makeVector<float>( PX[2], 0, -PX[0] );
            A[1] = makeVector<float>( 0, PX[2], -PX[1] );
            A *= f / (PX[2] * PX[2]);

            Matrix<3,3,float> rotjac;
            rotjac.T()[0] = SO3<float>::generator_field( 0, PX );
            rotjac.T()[1] = SO3<float>::generator_field( 1, PX );
            rotjac.T()[2] = SO3<float>::generator_field( 2, PX );
            
            Matrix<3,6,float> J;
            J.slice<0,0,3,3>() = -pose.get_rotation().get_matrix();  // jacobians for center
            J.slice<0,3,3,3>() = rotjac;      // jacobians for rotation
            
            Matrix<2,6,float> Fn = A * J;
            
            Vector<2,float> x = f * project(PX) + center;
            Vector<2,float> pos = feature->location;
            Vector<2,float> e = pos - x;
            
            FtF += Fn.T() * Fn;
            Fte += Fn.T() * e;
            
            toterr += e*e;
        }
        
        float avg_diag = 0;
        for ( int i = 0; i < 6; i++ ) {
            avg_diag += FtF(i,i);
        }
        avg_diag /= 6;
        
        for ( int i = 0; i < 6; i++ ) {
            FtF(i,i) = FtF(i,i) + lambda * avg_diag;
        }            
        
        Lapack_Cholesky<6,float> chol( FtF );
        Vector<6,float> soln = chol.backsub( Fte );

        Vector<3> old_center = -( pose.get_rotation().inverse() * pose.get_translation() );
        SE3<> newpose;
        newpose.get_rotation() = SO3<>( soln.slice<3,3>() ) * pose.get_rotation();
        Vector<3> new_center = old_center + soln.slice<0,3>();
        newpose.get_translation() = -( newpose.get_rotation() * new_center );
        
        // calculate new error
        float newerr = 0;
        for ( it = camera_in->features.begin(); it != camera_in->features.end(); it++ )
        {
            Feature *feature = (Feature*)it->second;
            
            Track *track = feature->track;
            if ( track == NULL ) continue;
            Point *point = track->point;
            if ( point == NULL ) continue;
            
            Vector<3,float> PX = newpose * project( point->position );
            
            Vector<2,float> x = f * project(PX) + center;
            Vector<2,float> pos = feature->location;
            Vector<2,float> e = pos - x;
            
            newerr += e*e;
        }
        
        if ( newerr < toterr ) {
            camera_in->node->pose = newpose;
            return true;
        }
        
        return false;
    }
    
    void Localizer::refinePoseLM( Camera *camera_in, int niter )
    {
        float lambda = 1e-3f;
        SE3<> last_pose = camera_in->node->pose;
        for ( int i = 0; i < niter; i++ )
        {
            // LM Step
            for ( int j = 0; j < 5; j++ ) 
            {
                bool good = refinePose( camera_in, lambda );
                if ( good ) {
                    if ( lambda > 1e-6 ) lambda /= 10.f;
                    break;
                } else {
                    if ( lambda < 1e6 ) lambda *= 10.f;
                }
            }
            
            if ( norm( (camera_in->node->pose*last_pose.inverse()).ln() ) < 1e-3 ) break;
            last_pose = camera_in->node->pose;
        }

    }
}
