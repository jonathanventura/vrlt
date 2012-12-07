/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: robustlsq.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/robustlsq.h>

#include <TooN/Lapack_Cholesky.h>
#include <TooN/Cholesky.h>

namespace vrlt {

    using namespace std;
    using namespace TooN;

    // m-estimator reference: "Robust Regression" by John Fox
    // NB: this will re-order the vector
    static float computeRobustStdDev( vector<float> &residualsqs )
    {
        vector<float>::iterator begin = residualsqs.begin();
        vector<float>::iterator end = residualsqs.end();
        vector<float>::iterator middle = begin + (end-begin)/2;
        partial_sort( begin, middle, end );
        float median = *middle;
        
        return ( median / ( 0.6745 * 0.6745 ) );
    }
    
    // this is the "k^2" parameter in Tukey
    static float computeTukeyParamSq( float var )
    {
        return ( 4.685 * 4.685 ) * var;
    }
    
    static float computeTukeyWeight( float ksq, float residsq )
    {
        if ( residsq > ksq ) return 0;
        float val = 1.f - (residsq/ksq);
        return val*val;
    }

    bool RobustLeastSq::updatePose( Camera *camera_in, int count, int iter, float eps )
    {
        Matrix<6> FtF = Zeros;
        Vector<6> Fte = Zeros;

        SE3<> pose( camera_in->node->pose );
        float f = camera_in->calibration->focal;
        Vector<2,float> center = camera_in->calibration->center;

        float e_scale = 0.5f;

        ElementList::iterator it;

        if ( iter < niter/2 )
        {
            vector<float> residualsqs;
            for ( it = root->points.begin(); it != root->points.end(); it++ )
            {
                Point *point = (Point*)it->second;
                if ( !point->tracked ) continue;

                Vector<3,float> PX = pose * project( point->position );

                Vector<2,float> x = f * project(PX) + center;
                Vector<2,float> e = point->location - x;
                e = e_scale * e;

                residualsqs.push_back( sqrtf(e*e) );
            }
            if ( residualsqs.size() > 0 ) {
                float var = computeRobustStdDev( residualsqs );
                ksq = computeTukeyParamSq( var );
            }
        }

        float toterr = 0.f;

        vector<float> weights( root->points.size() );

        int i = 0;
        for ( it = root->points.begin(); it != root->points.end(); it++,i++ )
        {
            Point *point = (Point*)it->second;
            if ( !point->tracked ) continue;

            Vector<3,float> PX = pose * project( point->position );

            Matrix<2,3,float> A;
            A[0] = makeVector<float>( PX[2], 0, -PX[0] );
            A[1] = makeVector<float>( 0, PX[2], -PX[1] );
            A *= f / (PX[2] * PX[2]);

            Matrix<3,6,float> J;

            for ( int m = 0; m < 6; m++ ) {
                J.T()[m] = SE3<float>::generator_field( m, unproject(PX) ).slice<0,3>();
            }

            Matrix<2,6,float> Fn = A * J;

            Vector<2,float> x = f * project(PX) + center;

            Vector<2,float> pos = point->location;
            Vector<2,float> e = pos - x;

            Vector<2,float> scaled_e = e_scale*e;
            float residsq = scaled_e*scaled_e;
            float w = computeTukeyWeight( ksq, residsq );
            weights[i] = w;

            FtF += Fn.T() * ( weights[i] * Fn );
            Fte += Fn.T() * ( weights[i] * e );

            toterr += (weights[i]*e)*(weights[i]*e);
        }

        float avg_diag = 0;
        for ( int i = 0; i < 6; i++ ) {
            avg_diag += FtF(i,i);
        }
        avg_diag /= 6;

        for ( int i = 0; i < 6; i++ ) {
            FtF(i,i) = FtF(i,i) + eps * avg_diag;
        }

        Lapack_Cholesky<6,float> chol( FtF );
        Vector<6,float> soln = chol.backsub( Fte );
        SE3<> newpose = SE3<>( soln ) * pose;

        // calculate new error
        float newerr = 0;
        i = 0;
        for ( it = root->points.begin(); it != root->points.end(); it++,i++ )
        {
            Point *point = (Point*)it->second;
            if ( !point->tracked ) continue;

            Vector<3,float> PX = newpose * project( point->position );

            Vector<2,float> x = f * project(PX) + center;

            Vector<2,float> pos = point->location;
            Vector<2,float> e = pos - x;

            if ( weights[i] == 0 ) {
                point->tracked = false;
            }

            newerr += (weights[i]*e)*(weights[i]*e);

            if ( iter == niter-1 ) {
                point->tracked = ( weights[i] > 0 );
            }
        }

        if ( newerr < toterr ) {
            camera_in->node->pose = newpose;
            return true;
        }

        return false;
    }

    bool RobustLeastSq::run( Camera *camera_in, int count )
    {
        float eps = 1e-3;
        int iter = 0;
        bool good_once = false;
        for ( int i = 0; i < niter; i++ ) {
            bool good = updatePose( camera_in, count, iter++, eps );
            if ( !good ) eps *= 10.;
            else good_once = true;
        }
        return good_once;
    }

}
