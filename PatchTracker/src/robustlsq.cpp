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

#include <Eigen/Eigen>

namespace vrlt
{
    // m-estimator reference: "Robust Regression" by John Fox
    // NB: this will re-order the vector
    static float computeRobustStdDev( std::vector<float> &residualsqs )
    {
        std::vector<float>::iterator begin = residualsqs.begin();
        std::vector<float>::iterator end = residualsqs.end();
        std::vector<float>::iterator middle = begin + (end-begin)/2;
        std::partial_sort( begin, middle, end );
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
  
    static double computeTukeyObjectiveFunction( double ksq, double residsq )
    {
      if ( residsq > ksq ) return ksq / 6.;
      return ( ksq / 6. ) * ( 1. - pow( 1. - (residsq/ksq), 3. ) );
    }

    bool RobustLeastSq::updatePose( Camera *camera_in, int iter, float eps )
    {
        Eigen::Matrix<float,6,6> FtF = Eigen::Matrix<float,6,6>::Zero();
        Eigen::Matrix<float,6,1> Fte = Eigen::Matrix<float,6,1>::Zero();

        Sophus::SE3f pose( camera_in->node->pose.cast<float>() );
        float f = camera_in->calibration->focal;
        Eigen::Vector2f center = camera_in->calibration->center.cast<float>();

        ElementList::iterator it;

        if ( iter < niter/2 )
        {
            std::vector<float> residualsqs;
            for ( it = root->points.begin(); it != root->points.end(); it++ )
            {
                Point *point = (Point*)it->second;
                if ( !point->tracked ) continue;

                Eigen::Vector3f PX = pose * (point->position.head(3).cast<float>()/(float)point->position[3]);

                Eigen::Vector2f x = f * project(PX) + center;
                Eigen::Vector2f e = point->location - x;
                
                residualsqs.push_back( e.dot(e) );
            }
            if ( residualsqs.size() > 0 ) {
                float var = computeRobustStdDev( residualsqs );
                ksq = computeTukeyParamSq( var );
            }
        }

        float toterr = 0.f;

        std::vector<float> weights( root->points.size() );

        int i = 0;
        for ( it = root->points.begin(); it != root->points.end(); it++,i++ )
        {
            Point *point = (Point*)it->second;
            if ( !point->tracked ) continue;

            Eigen::Vector3f PX = pose * (point->position.head(3).cast<float>()/(float)point->position[3]);

            Eigen::Matrix<float,2,3> A;
            A << PX[2], 0, -PX[0],
                0, PX[2], -PX[1];
            A *= f / (PX[2] * PX[2]);

            Eigen::Matrix<float,3,6> J;
            
            for ( int m = 0; m < 6; m++ ) {
                J.col(m) = ( Sophus::SE3f::generator( m ) * unproject(PX) ).head(3);
            }
            
            Eigen::Matrix<float,2,6> Fn = A * J;

            Eigen::Vector2f x = f * project(PX) + center;

            Eigen::Vector2f pos = point->location;
            Eigen::Vector2f e = pos - x;

            float residsq = e.dot(e);
            float w = computeTukeyWeight( ksq, residsq );
            weights[i] = w;

            FtF += Fn.transpose() * ( weights[i] * Fn );
            Fte += Fn.transpose() * ( weights[i] * e );

            toterr += computeTukeyObjectiveFunction( ksq, residsq );
        }

        float avg_diag = 0;
        for ( int i = 0; i < 6; i++ ) {
            avg_diag += FtF(i,i);
        }
        avg_diag /= 6;

        for ( int i = 0; i < 6; i++ ) {
            FtF(i,i) = FtF(i,i) + eps * avg_diag;
        }

        Eigen::Matrix<float,6,1> soln = FtF.llt().solve( Fte );
        Sophus::SE3f newpose = Sophus::SE3f::exp( soln ) * pose;

        // calculate new error
        float newerr = 0;
        i = 0;
        for ( it = root->points.begin(); it != root->points.end(); it++,i++ )
        {
            Point *point = (Point*)it->second;
            if ( !point->tracked ) continue;

            Eigen::Vector3f PX = newpose * (point->position.head(3).cast<float>()/(float)point->position[3]);

            Eigen::Vector2f x = f * project(PX) + center;

            Eigen::Vector2f pos = point->location;
            Eigen::Vector2f e = pos - x;

//            if ( weights[i] == 0 ) {
//                point->tracked = false;
//            }

            float residsq = e.dot(e);
            newerr += computeTukeyObjectiveFunction( ksq, residsq );

            if ( iter == niter-1 ) {
                point->tracked = ( weights[i] > 0 );
            }
        }

        if ( newerr < toterr ) {
            camera_in->node->pose = newpose.cast<double>();
            return true;
        }

        return false;
    }

    bool RobustLeastSq::run( Camera *camera_in )
    {
        float eps = 1e-3;
        int iter = 0;
        bool good_once = false;
        for ( int i = 0; i < niter; i++ ) {
            bool good = updatePose( camera_in, iter++, eps );
            if ( !good ) eps *= 10.;
            else good_once = true;
        }
        return good_once;
    }
    
}
