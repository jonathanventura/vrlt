/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: bundle.cpp
 * Author: Jonathan Ventura
 * Last Modified: 20.04.2014
 */

#include <BundleAdjustment/updatepose.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>

namespace vrlt
{
    struct ReprojectionError
    {
        ReprojectionError( const Eigen::Vector3d &_point, double _focal, double _x, double _y )
        : focal(_focal), x(_x), y(_y)
        {
            Eigen::Map<Eigen::Vector3d> pointvec(point);
            pointvec = _point;
        }
        
        template <typename T>
        bool operator()(const T* const camera,
                        T* residuals) const
        {
            // camera pose
            // camera[3,4,5] are the angle-axis rotation.
            T mypoint[3];
            mypoint[0] = T(point[0]);
            mypoint[1] = T(point[1]);
            mypoint[2] = T(point[2]);
            T p[3];
            ceres::AngleAxisRotatePoint(camera+3, mypoint, p);
            // camera[0,1,2] are the translation.
            p[0] += camera[0]; p[1] += camera[1]; p[2] += camera[2];
            
            // projection
            T xp = p[0] / p[2];
            T yp = p[1] / p[2];
            
            // intrinsics
            T fxp = T(focal) * xp;
            T fyp = T(focal) * yp;
            
            // residuals
            residuals[0] = fxp - T(x);
            residuals[1] = fyp - T(y);
            
            return true;
        }
        
        double point[3];
        double focal, x, y;
    };
    
    bool updatePose( Node *root, Camera *camera )
    {
        ceres::Problem problem;
        
        Node *node = camera->node;
        
        double params[6];
        
        Eigen::Map<Eigen::Vector3d> translationvec(params);
        translationvec = node->pose.translation();
        
        ceres::RotationMatrixToAngleAxis( node->pose.so3().matrix().data(), params+3 );

        ceres::LossFunction *lossFunction = new ceres::HuberLoss( 4.0 );
        
        Calibration *calibration = camera->calibration;
        
        ElementList::iterator pointit;
        for ( pointit = root->points.begin(); pointit != root->points.end(); pointit++ )
        {
            Point *point = (Point*)pointit->second;
            if ( !point->tracked ) continue;
            
            Eigen::Vector3d XYZ = project(point->position);
            
            ReprojectionError *reproj_error = new ReprojectionError(XYZ,
                                                                    calibration->focal,
                                                                    point->location[0]-calibration->center[0],
                                                                    point->location[1]-calibration->center[1]);
            
            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(reproj_error);
            problem.AddResidualBlock(cost_function, lossFunction, params );
        }
        
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
//        std::cout << summary.FullReport() << "\n";
        bool success = ( summary.termination_type != ceres::FAILURE );
        if ( success )
        {
            node->pose.translation() = translationvec;
            
            Eigen::Matrix3d R;
            ceres::AngleAxisToRotationMatrix(params+3, R.data());
            node->pose.so3() = Sophus::SO3d(R);
        }
        
        // update tracked flag
        for ( pointit = root->points.begin(); pointit != root->points.end(); pointit++ )
        {
            Point *point = (Point*)pointit->second;
            if ( !point->tracked ) continue;
            
            // check re-projection error
            Eigen::Vector2d proj = calibration->focal * project( node->pose * project(point->position) ) + calibration->center;
            Eigen::Vector2d diff = proj - point->location.cast<double>();
            double err = diff.norm();
            if ( err > 16.0 )
            {
                point->tracked = false;
            }
        }
        return success;
    }
}
