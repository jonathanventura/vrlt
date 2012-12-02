/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: estimator.cpp
 * Author: Jonathan Ventura
 * Last Modified: 2.12.2012
 */

#include <Estimator/estimator.h>

#include <TooN/SVD.h>

#include <algorithm>
#include <ext/algorithm>

#include <tag/threepointpose.h>
#include <tag/five_point.h>
#include <tag/ransac_estimators.h>

#ifdef USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#endif

#ifdef USE_DISPATCH
#include <dispatch/dispatch.h>
#endif

namespace vrlt {
    
    using namespace std;
    using namespace TooN;
    
    
    Vector<2> computeReprojError( Feature *feature )
    {
        Point *point = feature->track->point;
        Camera *camera = feature->camera;
        Calibration *calibration = camera->calibration;
        
        Vector<2> err;
        
        switch ( calibration->type )
        {
            case Calibration::Perspective:
            {
                Vector<2> projpt = camera->calibration->project( project( camera->node->globalPose() * project(point->position ) ) );
                err = projpt - feature->location;
                break;
            }
                
            case Calibration::Spherical:
            {
                Vector<3> projray = camera->node->globalPose() * project( point->position );
                Vector<3> obsray = calibration->unproject( feature->location );
                err[0] = acos( projray * obsray / norm(projray) / norm(obsray) ) * 180. / M_PI;
                err[1] = 0;
                break;
            }
                
            default:
            {
                err = Zeros;
            }
        }
        
        return err;
    }
    
    Vector<3> applyPose( const SE3<> &pose, const Vector<4> &point )
    {
        return pose.get_rotation() * point.slice<0,3>() + point[3] * pose.get_translation();
    }
    
    Vector<4> triangulate( const TooN::SE3<> &pose, PointPair point_pair )
    {
        Matrix<3,4> P;
        P.slice<0,0,3,3>() = pose.get_rotation().get_matrix();
        P.T()[3] = pose.get_translation();
        
        Matrix<4> J;
        SVD<4> svdJ;
        
        Vector<2> point0 = project( point_pair.first );
        Vector<2> point1 = project( point_pair.second );
        
        J[0] = makeVector( -1, 0, point0[0], 0 );
        J[1] = makeVector( 0, -1, point0[1], 0 );
        J[2] = P[2] * point1[0] - P[0];
        J[3] = P[2] * point1[1] - P[1];
        
        svdJ.compute( J );
        
        return svdJ.get_VT()[3];
    }
    
    Vector<4> triangulate( Feature *feature1, Feature *feature2 )
    {
        Camera *camera1 = feature1->camera;
        Camera *camera2 = feature2->camera;
        
        SE3<> pose1 = camera1->node->globalPose();
        SE3<> pose2 = camera2->node->globalPose();
        
        SE3<> rel_pose = pose2 * pose1.inverse();
        
        PointPair point_pair;
        point_pair.first = feature1->unproject();
        point_pair.second = feature2->unproject();
        
        Vector<4> pt = triangulate( rel_pose, point_pair );
        
        return pose1.inverse() * pt;
    }
    
    void triangulate( Node *root, Track *track )
    {
        vector< Matrix<3,4> > poses;
        vector< Vector<3> > obs;
        
        ElementList::iterator it;
        for ( it = track->features.begin(); it != track->features.end(); it++ )
        {
            Feature *feature = (Feature *)it->second;
            if ( feature->camera->node->root() != root ) continue;
            obs.push_back( feature->unproject() );
            
            SE3<> pose = feature->camera->node->globalPose();
            Matrix<3,4> P;
            P.slice<0,0,3,3>() = pose.get_rotation().get_matrix();
            P.T()[3] = pose.get_translation();

            poses.push_back( P );
        }
        
        int N = poses.size();

        if ( N < 2 ) return;
        Matrix<> J( 3 * N, 4 );
        
        int row = 0;
        for ( int i = 0; i < N; i++ )
        {
            Vector<3> X = obs[i];
            Matrix<3> skew;
            skew[0] = makeVector( 0, X[2], -X[1] );
            skew[1] = makeVector( -X[2], 0, X[0] );
            skew[2] = makeVector( X[1], -X[0], 0 );
            Matrix<3,4> Jmat = skew * poses[i];;
            J[row++] = Jmat[0];
            J[row++] = Jmat[1];
            J[row++] = Jmat[2];
        }
        
        SVD<> svdJ( J );
        track->point->position = svdJ.get_VT()[3];
    }

    int Estimator::sampleSize()
    {
        return 1;
    }
    
    int Estimator::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        return 0;
    }
    
    void Estimator::chooseSolution( int soln )
    {
        
    }
    
    double Estimator::score( PointPairList::iterator it )
    {
        return 0;
    }
    
    bool Estimator::canRefine()
    {
        return true;
    }
    
    int FivePointEssential::sampleSize()
    {
        return 5;
    }
    
    int FivePointEssential::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        tr1::array< pair< Vector<3>, Vector<3> >, 5> points;
        int n = 0;
        PointPairList::iterator it;
        for ( it = begin; n < 5; it++, n++ )
        {
            points[n] = *it;
        }
        Elist = tag::five_point( points );
        return (int)Elist.size();
    }
    
    void FivePointEssential::chooseSolution( int soln )
    {
        E = Elist[soln];
    }
    
    double FivePointEssential::score( PointPairList::iterator it )
    {
        Vector<3> x1 = it->first;
        Vector<3> x2 = it->second;
        
        x2 /= x2[2];
        Vector<3> line = E * x1;
        double d = x2 * line;
        return (d*d) / (line[0]*line[0] + line[1]*line[1]);
    }
    
    SE3<> FivePointEssential::getPose( PointPairList::iterator begin, PointPairList::iterator end )
    {
        vector< SE3<> > poses = tag::se3_from_E( E );
        
        SE3<> bestpose;
        int bestcount = 0;
        for ( int i = 0; i < poses.size(); i++ ) {
            SE3<> pose = poses[i];
            
            Vector<3> newup = pose.get_rotation() * makeVector( 0, 1, 0 );
            if ( newup[1] < 0 ) continue;

            Vector<3> old_trans = pose.get_translation();
            pose.get_translation() = old_trans / norm(old_trans);
            
            int count = 0;
            
            PointPairList::iterator it;
            for ( it = begin; it != end; it++ ) 
            {
                Vector<4> X = triangulate( pose, *it );
                Vector<3> Xproj = project(X);
                
                if ( Xproj * it->first > 0 ) count++;
            }
            
            if ( count > bestcount ) {
                bestcount = count;
                bestpose = pose;
            }
        }

        PointPairList mylist( begin, end );
        
        bestpose = tag::optimize_epipolar( mylist, bestpose );
        
        return bestpose;
    }
    
    bool FivePointEssential::canRefine()
    {
        return false;
    }
    
    int UprightEssential::sampleSize()
    {
        return 5;
    }
    
    int UprightEssential::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        int N = (int) distance( begin, end );
        Matrix<> A( N, 6 );
        int n = 0;
        PointPairList::iterator it;
        for ( it = begin; it != end; it++,n++ )
        {
            Vector<3> x1 = it->first;
            Vector<3> x2 = it->second;
            
            A[n] = makeVector( x1[2]*x2[2]+x1[0]*x2[0], x2[0]*x1[1], x2[0]*x1[2]-x1[0]*x2[2], x1[0]*x2[1], x2[1]*x1[2], x1[1]*x2[2] );
        }
        
        SVD<> svdA( A );

        Vector<6> sol = svdA.get_VT()[ svdA.get_VT().num_rows() - 1 ];
        
        E[0] = makeVector( sol[0],  sol[1], sol[2] );
        E[1] = makeVector( sol[3],  0,      sol[4] );
        E[2] = makeVector( -sol[2], sol[5], sol[0] );
        
        SVD<3> svdE( E );
        E = svdE.get_U() * DiagonalMatrix<3>( makeVector( 1, 1, 0 ) ) * svdE.get_VT();
        
        return 1;
    }
    
    double UprightEssential::score( PointPairList::iterator it )
    {
        Vector<3> x1 = it->first;
        Vector<3> x2 = it->second;

        double scoresq = 0;
        
        switch ( scoreType )
        {
            case Pixel:
            {
                pair<double,double> scores = tag::essential_reprojection_errors_squared( E, x1, x2 );
                scoresq = scores.first;
                break;
            }
                
            case Angle:
            {
                Vector<3> line = E * x1;
                double angle = asin( x2 * line / norm(x2) / norm(line) ) * 180. / M_PI;
                scoresq = angle*angle;
                break;
            }
        }
        
        return scoresq;
    }
    
    SE3<> UprightEssential::getPose( PointPairList::iterator begin, PointPairList::iterator end )
    {
        vector< SE3<> > poses = tag::se3_from_E( E );
        
        SE3<> bestpose;
        int bestcount = 0;
        for ( int i = 0; i < poses.size(); i++ ) {
            SE3<> pose = poses[i];
            
            Vector<3> newup = pose.get_rotation() * makeVector( 0, 1, 0 );
            if ( newup[1] < 0 ) continue;

            Vector<3> old_rot = pose.get_rotation().ln();
            pose.get_rotation() = SO3<>::exp( makeVector( 0, old_rot[1], 0 ) );
            
            int count = 0;
            
            PointPairList::iterator it;
            for ( it = begin; it != end; it++ ) 
            {
                Vector<4> X = triangulate( pose, *it );
                Vector<3> Xproj = project(X);
                
                if ( Xproj * it->first > 0 ) count++;
            }
            
            if ( count > bestcount ) {
                bestcount = count;
                bestpose = pose;
            }
            
        }

        return bestpose;
    }
    
    int UprightPose::sampleSize()
    {
        return 3;
    }
    
    int UprightPose::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        int N = (int) distance( begin, end );
        int numrows = 2 * N;
        if ( N == 3 ) numrows = 5;
        Matrix<> A( numrows, 6 );
        int n = 0;
        PointPairList::iterator it;
        for ( it = begin; it != end; it++ )
        {
            Vector<3> X = it->first;
            Vector<3> x = it->second;
            
            A[n++] = makeVector( -x[2]*X[0]+X[2]*x[0], -x[2]*X[2]-X[0]*x[0], -x[2], 0, 0, x[0] );
            if ( n == numrows ) break;
            A[n++] = makeVector( X[2]*x[1], -X[0]*x[1], 0, -x[2]*X[1], -x[2], x[1] );
        }

        SVD<> svdA( A );
        Vector<6> sol = svdA.get_VT()[ svdA.get_VT().num_rows() - 1 ];
        if ( sol[3] == 0 ) return 0;
        sol = sol / sol[3];
        
        double factor = sqrt(sol[0]*sol[0]+sol[1]*sol[1]);
        sol[0] /= factor;
        sol[1] /= factor;
        
        Matrix<3> R;
        R[0] = makeVector( sol[0], 0, sol[1] );
        R[1] = makeVector( 0, sol[3], 0 );
        R[2] = makeVector( -sol[1], 0, sol[0] );
        
        pose.get_rotation() = SO3<>( R );
        pose.get_translation() = makeVector( sol[2], sol[4], sol[5] );
        
        return 1;
    }
    
    double UprightPose::score( PointPairList::iterator it )
    {
        double scoresq = 0;
        
        Vector<3> projray = pose * it->first;
        Vector<3> obsray = it->second;
        
        switch ( scoreType ) 
        {
            case Pixel:
            {
                Vector<2> diff = project(projray) - project(obsray);
                scoresq = diff*diff;
                break;
            }
                
            case Angle:
            {
                double angle = acos( projray * obsray / norm(projray) / norm(obsray) ) * 180. / M_PI;
                scoresq = angle*angle;
                break;
            }
        }
        
        return scoresq;
    }
    
    int SixPointPose::sampleSize()
    {
        return 6;
    }
    
    int SixPointPose::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        int N = distance( begin, end );
        
        Matrix<> A( 2*N, 12 );
        int n = 0;
        PointPairList::iterator it;
        for ( it = begin; it != end; it++ ) {
            Vector<3> X = it->first;
            Vector<2> x = project(it->second);
            
            A[n++] = makeVector( -X[0], -X[1], -X[2], 1,   0, 0, 0, 0,   x[0]*X[0], x[0]*X[1], x[0]*X[2], x[0] );
            A[n++] = makeVector( 0, 0, 0, 0,   -X[0], -X[1], -X[2], 1,   x[1]*X[0], x[1]*X[1], x[1]*X[2], x[1] );
        }
        
        poses.resize(2);
        
        SVD<> svdA( A );
        Vector<12> sol = svdA.get_VT()[11];
        
        Vector<12> mysol;
        Matrix<3> Rmat;
        Vector<3> t;
        
        mysol = sol / sol[5];
        Rmat[0] = mysol.slice<0,3>();
        Rmat[1] = mysol.slice<4,3>();
        Rmat[2] = mysol.slice<8,3>();
        t[0] = mysol[3];
        t[1] = mysol[7];
        t[2] = mysol[11];
        poses[0] = SE3<>( SO3<>(Rmat).ln(), t );
        
        mysol = sol / -sol[5];
        Rmat[0] = mysol.slice<0,3>();
        Rmat[1] = mysol.slice<4,3>();
        Rmat[2] = mysol.slice<8,3>();
        t[0] = mysol[3];
        t[1] = mysol[7];
        t[2] = mysol[11];
        poses[1] = SE3<>( SO3<>(Rmat).ln(), t );
        
        return 2;
    }
    
    void SixPointPose::chooseSolution( int soln )
    {
        pose = poses[soln];
        R0 = pose.get_rotation().get_matrix()[0];
        R1 = pose.get_rotation().get_matrix()[1];
        R2 = pose.get_rotation().get_matrix()[2];
        t = pose.get_translation();
    }
    
    double SixPointPose::score( PointPairList::iterator it )
    {
#ifdef USE_ACCELERATE
        Vector<3> X = it->first;
        Vector<2> x = it->second.slice<0,2>();
        double scale = 1. / it->second[2];
        vDSP_vsmulD( x.get_data_ptr(), 1, &scale, x.get_data_ptr(), 1, 2 );
        
        Vector<3> poseX;
        vDSP_dotprD( R0.get_data_ptr(), 1, X.get_data_ptr(), 1, poseX.get_data_ptr()    , 3 );
        vDSP_dotprD( R1.get_data_ptr(), 1, X.get_data_ptr(), 1, poseX.get_data_ptr() + 1, 3 );
        vDSP_dotprD( R2.get_data_ptr(), 1, X.get_data_ptr(), 1, poseX.get_data_ptr() + 2, 3 );
        vDSP_vaddD( poseX.get_data_ptr(), 1, t.get_data_ptr(), 1, poseX.get_data_ptr(), 1, 3 );
        
        double c;
        vDSP_dotprD( poseX.get_data_ptr(), 1, it->second.get_data_ptr(), 1, &c, 3 );
        if ( c < 0 ) return INFINITY;
        
        scale = 1. / poseX[2];
        
        Vector<2> xproj = poseX.slice<0,2>();
        vDSP_vsmulD( xproj.get_data_ptr(), 1, &scale, xproj.get_data_ptr(), 1, 2 );
        
        Vector<2> diff;
        vDSP_vsubD( xproj.get_data_ptr(), 1, x.get_data_ptr(), 1, diff.get_data_ptr(), 1, 2 );
        
        double s;
        vDSP_svesqD( diff.get_data_ptr(), 1, &s, 2 );
        
        return s;
#else
        Vector<3> poseX = pose * it->first;
        if ( poseX * it->second < 0 ) return INFINITY;
        
        Vector<2> xproj = project( poseX );
        Vector<2> diff = xproj - project( it->second );
        return diff*diff;
#endif
    }
    
    
    
    int ThreePointPlane::sampleSize()
    {
        return 3;
    }
    
    int ThreePointPlane::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        int N = distance(begin,end);
        Matrix<> A( N, 4 );
        
        int i = 0;
        for ( PointPairList::iterator it = begin; it != end; it++,i++ )
        {
            A[i] = unproject( it->first );
        }
        
        SVD<> svdA( A );
        Vector<4> sol = svdA.get_VT()[ svdA.get_VT().num_rows() - 1 ];
        
        plane = sol / norm(sol.slice<0,3>());
        
        return 1;
    }
    
    bool ThreePointPlane::canRefine()
    {
        return true;
    }
    
    double ThreePointPlane::score( PointPairList::iterator it )
    {
        // N * X + D = 0
        // N * ( X + offset ) + D = s
        // N * X + D + N * offset = s
        // N * offset = s
        
        Vector<4> pt = unproject( it->first );
        return fabs( plane * pt );
    }
    
    
    
    
    
    
    
    int ThreePointPose::sampleSize()
    {
        return 3;
    }
    
    int ThreePointPose::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        Vector<3> x[3];
        Vector<2> z[3];
        
        int n = 0;
        PointPairList::iterator it;
        for ( it = begin; n < 3; it++,n++ ) {
            x[n] = it->first;
            z[n] = it->second.slice<0,2>();
        }
        
        poses.clear();
        int nposes = tag::three_point_pose( x, z, poses );
        
        return nposes;
    }
    
    void ThreePointPose::chooseSolution( int soln )
    {
        pose = poses[soln];
        R0 = pose.get_rotation().get_matrix()[0];
        R1 = pose.get_rotation().get_matrix()[1];
        R2 = pose.get_rotation().get_matrix()[2];
        t = pose.get_translation();
    }
    
    double ThreePointPose::score( PointPairList::iterator it )
    {
#ifdef USE_ACCELERATE
        Vector<3> X = it->first;
        Vector<2> x = it->second.slice<0,2>();
        //double featurescale = it->second[2];
        double scale = 1. / it->second[2];
        vDSP_vsmulD( x.get_data_ptr(), 1, &scale, x.get_data_ptr(), 1, 2 );
        
        Vector<3> poseX;
        vDSP_dotprD( R0.get_data_ptr(), 1, X.get_data_ptr(), 1, poseX.get_data_ptr()    , 3 );
        vDSP_dotprD( R1.get_data_ptr(), 1, X.get_data_ptr(), 1, poseX.get_data_ptr() + 1, 3 );
        vDSP_dotprD( R2.get_data_ptr(), 1, X.get_data_ptr(), 1, poseX.get_data_ptr() + 2, 3 );
        vDSP_vaddD( poseX.get_data_ptr(), 1, t.get_data_ptr(), 1, poseX.get_data_ptr(), 1, 3 );
        
        double c;
        vDSP_dotprD( poseX.get_data_ptr(), 1, it->second.get_data_ptr(), 1, &c, 3 );
        if ( c < 0 ) return INFINITY;
        
        scale = 1. / poseX[2];
        
        Vector<2> xproj = poseX.slice<0,2>();
        vDSP_vsmulD( xproj.get_data_ptr(), 1, &scale, xproj.get_data_ptr(), 1, 2 );
        
        Vector<2> diff;
        vDSP_vsubD( xproj.get_data_ptr(), 1, x.get_data_ptr(), 1, diff.get_data_ptr(), 1, 2 );
        //diff /= featurescale;
        
        double s;
        vDSP_svesqD( diff.get_data_ptr(), 1, &s, 2 );
        
        return s;
#else
        Vector<3> poseX = pose * it->first;
        //if ( poseX[2] < 0 ) return INFINITY;
        if ( poseX * it->second < 0 ) return INFINITY;
        
        Vector<2> x = project( it->second );
        //Vector<2> x = it->second.slice<0,2>();
        //double featurescale = it->second[2];
        
        Vector<2> xproj = project( poseX );
        Vector<2> diff = xproj - x;
        //diff /= featurescale;
        return diff*diff;
#endif
    }
    
    bool ThreePointPose::canRefine()
    {
        return false;
    }

    int Homography::sampleSize()
    {
        return 4;
    }

    int Homography::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        int N = (int) distance( begin, end );
        Matrix<> A( 2*N, 9 );
        int n = 0;
        PointPairList::iterator it;
        for ( it = begin; it != end; it++ )
        {
            Vector<3> x1 = it->first;
            Vector<3> x2 = it->second;
            
            A[n++] = makeVector( x2[2]*x1[0], x2[2]*x1[1], x2[2]*x1[2], 0, 0, 0, x2[0]*x1[0], x2[0]*x1[1], x2[0]*x1[2] );
            A[n++] = makeVector( 0, 0, 0, x2[2]*x1[0], x2[2]*x1[1], x2[2]*x1[2], x2[1]*x1[0], x2[1]*x1[1], x2[1]*x1[2] );
        }
        
        SVD<> svdA( A );
        Vector<9> sol = svdA.get_VT()[ svdA.get_VT().num_rows() - 1 ];
        
        H[0] = sol.slice<0,3>();
        H[1] = sol.slice<3,3>();
        H[2] = sol.slice<6,3>();
        
        return 1;
    }
    
    double Homography::score( PointPairList::iterator it )
    {
        Vector<3> x1 = it->first;
        Vector<3> x2 = it->second;
        
        Vector<3> diff = H * x1 - x2;
        return diff * diff;
    }
    
    
    
    
    int VerticalVanishingPoint::sampleSize()
    {
        return 2;
    }
    
    int VerticalVanishingPoint::compute( PointPairList::iterator begin, PointPairList::iterator end )
    {
        int N = (int) distance( begin, end );
        
        Matrix<> A( N, 3 );
        int n = 0;
        PointPairList::iterator it;
        for ( it = begin; it != end; it++ )
        {
            Vector<3> coeffs = it->first;
            A[n++] = coeffs;
        }
        
        if ( N == 2 ) {
            Vector<3> coeffs1 = A[0];
            Vector<3> coeffs2 = A[1];
            vp = coeffs1 ^ coeffs2;
        } else {
            SVD<> svdA( A );
            vp = svdA.get_VT()[ svdA.get_VT().num_rows() - 1 ];
        }
        normalize( vp );
        
        // ensure pointing up
        if ( vp[1] < 0 ) vp = -vp;
        
        // find rotation axis to make vertical
        Vector<3> axis = vp ^ makeVector(0,1,0);
        normalize( axis );
        
        // find angle
        double angle = acos( vp[1] );
        
        // get rotation
        R = SO3<>( axis * angle );
        
        return 1;
    }
    
    double VerticalVanishingPoint::score( PointPairList::iterator it )
    {
        Vector<3> coeffs = it->first;
        
        double angle = asin( coeffs * vp ) * 180. / M_PI;
        return angle * angle;
    }
    
    bool VerticalVanishingPoint::canRefine()
    {
        return true;
    }

    
    

        
    
    PROSAC::PROSAC() :
    num_trials( 20000 ),
    inlier_threshold( 0.01 ),
    min_num_inliers( 100 )
    {
        
    }
    
    struct InlierData
    {
        double threshsq;
        PointPairList::iterator begin;
        Estimator &estimator;
        vector<bool> &inliers;
        InlierData( double _threshsq, PointPairList::iterator _begin, Estimator &_estimator, vector<bool> &_inliers )
        : threshsq( _threshsq ), begin( _begin ), estimator( _estimator ), inliers( _inliers ) { }
    };

    static void inlierFn( void *context, size_t n )
    {
        InlierData *inlierData = (InlierData*)context;
        inlierData->inliers[n] = ( inlierData->estimator.score( inlierData->begin+n ) < inlierData->threshsq );
    }
    
    int PROSAC::countInliers( PointPairList::iterator begin, PointPairList::iterator end, Estimator &estimator )
    {
        int N = (int) distance( begin, end );
        double threshsq = inlier_threshold * inlier_threshold;
        vector<bool> inliers( N );
        PointPairList::iterator it;
        InlierData inlierData( threshsq, begin, estimator, inliers );
#ifdef USE_DISPATCH
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
        dispatch_apply_f( N, queue, &inlierData, inlierFn );
#else
        for ( int i = 0; i < N; i++ ) inlierFn( &inlierData, i );
#endif
        int count = 0;
        for ( int n = 0; n < N; n++ )
        {
            if ( inliers[n] ) count++;
        }
        return count;
    }
    
    int PROSAC::countInliers( PointPairList::iterator begin, PointPairList::iterator end, Estimator &estimator, std::vector<bool> &inliers )
    {
        int N = (int) distance( begin, end );
        double threshsq = inlier_threshold * inlier_threshold;
        inliers.resize( N );
        PointPairList::iterator it;
        InlierData inlierData( threshsq, begin, estimator, inliers );
#ifdef USE_DISPATCH
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
        dispatch_apply_f( N, queue, &inlierData, inlierFn );
#else
        for ( int i = 0; i < N; i++ ) inlierFn( &inlierData, i );
#endif
//        int n = 0;
//        int count = 0;
//        for ( it = begin; it != end; it++,n++ ) {
//            bool good = ( estimator.score( it ) < threshsq );
//            inliers[n] = good;
//            if ( good ) count++;
//        }
        int count = 0;
        for ( int n = 0; n < N; n++ )
        {
            if ( inliers[n] ) count++;
        }
        return count;
    }

    int PROSAC::countInliers( PointPairList::iterator begin, PointPairList::iterator end, int step, Estimator &estimator, std::vector<bool> &inliers )
    {
        int N = (int) distance( begin, end );
        double threshsq = inlier_threshold * inlier_threshold;
        inliers.resize( N );
        PointPairList::iterator it;
        InlierData inlierData( threshsq, begin, estimator, inliers );
#ifdef USE_DISPATCH
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
        dispatch_apply_f( N, queue, &inlierData, inlierFn );
#else
        for ( int i = 0; i < N; i++ ) inlierFn( &inlierData, i );
#endif
        //        int n = 0;
        //        int count = 0;
        //        for ( it = begin; it != end; it++,n++ ) {
        //            bool good = ( estimator.score( it ) < threshsq );
        //            inliers[n] = good;
        //            if ( good ) count++;
        //        }
        int count = 0;
        for ( int n = 0; n < N; n+=step )
        {
            // only count one inlier for each set of *step* matches
            for ( int i = 0; i < step; i++ ) {
                if ( inliers[n+i] ) {
                    count++;
                    break;
                }
            }
        }
        return count;
    }

    int PROSAC::compute( PointPairList::iterator begin, PointPairList::iterator end, Estimator &estimator, std::vector<bool> &inliers )
    {
        int bestNInliers = 0;
        
        // number of samples that must match for acceptable estimate
        int Kthresh = min_num_inliers;  
        
        //double w = 0.05;
        //int Kthresh = std::max( 3, (int)ceil(w * matches.size()) );
        
        PointPairList random_subset( estimator.sampleSize() );
        PointPairList best_subset( estimator.sampleSize() );
        int best_soln = 0;
        
        const int m = estimator.sampleSize();
        const int N = (int) distance(begin,end);
        const float T_N = 200000.f;
        int n = estimator.sampleSize();
        float T_n = T_N;
        for ( int i=0; i<m; ++i ) T_n *= (float)(n-i)/(N-i);
        
        unsigned n_star = N;
        unsigned T_n_prime = 1;
        unsigned max_K = 0;
        
        if ( N < estimator.sampleSize() )
        {
            inliers.resize(N);
            for ( int i = 0; i < N; i++ ) inliers[i] = false;
            return 0;
        }
        
        for ( int t = 1 ; t <= num_trials ; ++t )
        {    	
            if( (t > T_n_prime) && (n < n_star) ) {
                float T_nplus1 = (T_n * (n+1)) / (n+1-m);       // Eq.(3)
                T_n_prime = T_n_prime + ceil(T_nplus1 - T_n);  // Eq.(4)
                T_n = T_nplus1;
                n++;
            }
            
            if (T_n_prime < t) {
                random_sample(begin, begin + n, random_subset.begin(), random_subset.begin() + m );
            }
            else {
                random_sample(begin, begin + n-1, random_subset.begin(), random_subset.begin() + m - 1 );
                random_subset[m-1] = *(begin+(n-1));
            }
            
            // estimate model M, given the subset S
            int nsolns = estimator.compute( random_subset.begin(), random_subset.end() );
            if ( nsolns == 0 ) continue;
            
            // find out "consensus set" S* that fits estimate M
            for ( int soln = 0; soln < nsolns; soln++ )
            {
                estimator.chooseSolution( soln );
                int K = countInliers( begin, end, estimator, inliers );
                
                // if K big enough and better than any model before, store result
                if ( K > max_K )
                {
                    max_K = K;
                    best_subset = random_subset;
                    best_soln = soln;
                    bestNInliers = K;
                }
            }
            if ( bestNInliers >= Kthresh ) break;
        }
        
        int nsolns = estimator.compute( best_subset.begin(), best_subset.end() );
        if ( nsolns == 0 ) return 0;
        estimator.chooseSolution( best_soln );
        bestNInliers = countInliers( begin, end, estimator, inliers );

        if ( estimator.canRefine() && bestNInliers > estimator.sampleSize() ) {
            PointPairList bestlist;
            n = 0;
            PointPairList::iterator it;
            for ( it = begin; it != end; it++,n++ ) {
                if ( inliers[n] ) bestlist.push_back( *it );
            }
            int nsolns = estimator.compute( bestlist.begin(), bestlist.end() );
            assert( nsolns == 1 );
            bestNInliers = countInliers( begin, end, estimator, inliers );
        }
        
        return bestNInliers;
    }

    int PROSAC::compute( PointPairList::iterator begin, PointPairList::iterator end, PointPairList::iterator all_begin, PointPairList::iterator all_end, Estimator &estimator, std::vector<bool> &inliers )
    {
        int bestNInliers = 0;
        
//        int step = distance(all_begin,all_end)/distance(begin,end);
        
        // number of samples that must match for acceptable estimate
        int Kthresh = min_num_inliers;  
        
        //double w = 0.05;
        //int Kthresh = std::max( 3, (int)ceil(w * matches.size()) );
        
        PointPairList random_subset( estimator.sampleSize() );
        PointPairList best_subset( estimator.sampleSize() );
        int best_soln = 0;
        
        const int m = estimator.sampleSize();
        const int N = (int) distance(begin,end);
        const float T_N = 200000.f;
        int n = estimator.sampleSize();
        float T_n = T_N;
        for ( int i=0; i<m; ++i ) T_n *= (float)(n-i)/(N-i);
        
        unsigned n_star = N;
        unsigned T_n_prime = 1;
        unsigned max_K = 0;
        
        if ( N < estimator.sampleSize() )
        {
            inliers.resize(N);
            for ( int i = 0; i < N; i++ ) inliers[i] = false;
            return 0;
        }
        
        for ( int t = 1 ; t <= num_trials ; ++t )
        {    	
            if( (t > T_n_prime) && (n < n_star) ) {
                float T_nplus1 = (T_n * (n+1)) / (n+1-m);       // Eq.(3)
                T_n_prime = T_n_prime + ceil(T_nplus1 - T_n);  // Eq.(4)
                T_n = T_nplus1;
                n++;
            }
            
            if (T_n_prime < t) {
                random_sample(begin, begin + n, random_subset.begin(), random_subset.begin() + m );
            }
            else {
                random_sample(begin, begin + n-1, random_subset.begin(), random_subset.begin() + m - 1 );
                random_subset[m-1] = *(begin+(n-1));
            }
            
            // estimate model M, given the subset S
            int nsolns = estimator.compute( random_subset.begin(), random_subset.end() );
            if ( nsolns == 0 ) continue;
            
            // find out "consensus set" S* that fits estimate M
            for ( int soln = 0; soln < nsolns; soln++ )
            {
                estimator.chooseSolution( soln );
                int K = countInliers( all_begin, all_end, estimator, inliers );
//                int K = countInliers( all_begin, all_end, step, estimator, inliers );
                
                // if K big enough and better than any model before, store result
                if ( K > max_K )
                {
                    max_K = K;
                    best_subset = random_subset;
                    best_soln = soln;
                    bestNInliers = K;
                }
            }
            if ( bestNInliers >= Kthresh ) break;
        }
        
        int nsolns = estimator.compute( best_subset.begin(), best_subset.end() );
        if ( nsolns == 0 ) return 0;
        estimator.chooseSolution( best_soln );
        bestNInliers = countInliers( all_begin, all_end, estimator, inliers );
//        bestNInliers = countInliers( all_begin, all_end, step, estimator, inliers );
        
        if ( estimator.canRefine() && bestNInliers > estimator.sampleSize() ) {
            PointPairList bestlist;
            n = 0;
            PointPairList::iterator it;
            for ( it = all_begin; it != all_end; it++,n++ ) {
                if ( inliers[n] ) bestlist.push_back( *it );
            }
            int nsolns = estimator.compute( bestlist.begin(), bestlist.end() );
            assert( nsolns == 1 );
            bestNInliers = countInliers( all_begin, all_end, estimator, inliers );
//            bestNInliers = countInliers( all_begin, all_end, step, estimator, inliers );
        }
        
        return bestNInliers;
    }
}
