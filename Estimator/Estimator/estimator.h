/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: estimator.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef __ESTIMATOR_H
#define __ESTIMATOR_H

#include <MultiView/multiview.h>

namespace vrlt {
    
    /** \addtogroup Estimator
     * \brief Robust estimation algorithms and utilities
     * @{
     */
    
    /** \brief 3D point pairs */
    typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > PointPair;
    /** \brief List of 3D point pairs */
    typedef std::vector<PointPair> PointPairList;

    /** Computes the reprojection error of a feature in its camera image. */
    Eigen::Vector2d computeReprojError( Feature *feature );
    /** Triangulates a point given a relative pose and a pair of homogeneous observations. */
    Eigen::Vector4d triangulate( const Sophus::SE3d &pose, PointPair point_pair );
    /** Triangulates a point given two features. */
    Eigen::Vector4d triangulate( Feature *feature1, Feature *feature2 );
    /** Triangulates a track using all feature observations. */
    void triangulate( Node *root, Track *track );
    
    /** \brief Generic estimation algorithm class
     *
     * This class is a base class for estimation algorithms which take
     * a list of 3D point pairs as input.  The algorithm can produce
     * multiple solutions.
     */
    struct Estimator
    {
        typedef enum {
            Pixel,
            Angle
        } ScoreType;
        ScoreType scoreType;
        /** Minimal sample size. */
        virtual int sampleSize();
        /** Computes solutions given a list of point pairs.
         * \return Number of solutions
         */
        virtual int compute( PointPairList::iterator begin, PointPairList::iterator end );
        /** Chooses which solution to consider in the score() function. */
        virtual void chooseSolution( int soln );
        /** Compute the score of a given observation using the chosen solution. */
        virtual double score( PointPairList::iterator it );
        /** Returns whether the compute() function can refine an estimate using all pairs in the list. */
        virtual bool canRefine();
        Estimator() : scoreType( Pixel ) { }
        virtual ~Estimator() { }
    };
    
    /** \brief Five point estimation
     *
     * This class implements the five point algorithm for
     * calibrated relative pose estimation.
     */
    struct FivePointEssential : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        double score( PointPairList::iterator it );
        std::vector< Eigen::Matrix3d > Elist;
        std::vector< Eigen::Matrix3d > Rlist;
        std::vector< Eigen::Vector3d > tlist;
        Eigen::Matrix3d E;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        void chooseSolution( int soln );
        virtual bool canRefine();
    };
    
    /** \brief Upright essential matrix estimation
     *
     * This class implements essential matrix estmation
     * using the upright constraint.
     */
    struct UprightEssential : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        double score( PointPairList::iterator it );
        Eigen::Matrix3d E;
        Sophus::SE3d getPose( PointPairList::iterator begin, PointPairList::iterator end );
    };
    
    /** \brief Three point pose
     *
     * Pose estimation from three points using a calibrated camera.
     */
    struct ThreePointPose : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        void chooseSolution( int soln );
        double score( PointPairList::iterator it );
        std::vector< Sophus::SE3d > poses;
        Sophus::SE3d pose;
        Eigen::Vector3d R0;
        Eigen::Vector3d R1;
        Eigen::Vector3d R2;
        Eigen::Vector3d t;
        bool canRefine();
        ThreePointPose() { poses.push_back( Sophus::SE3d() ); }
    };

    /** \brief Six point pose
     *
     * Linear pose estimation from six points using a calibrated camera.
     */
    struct SixPointPose : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        void chooseSolution( int soln );
        double score( PointPairList::iterator it );
        std::vector< Sophus::SE3d > poses;
        Sophus::SE3d pose;
        Eigen::Vector3d R0;
        Eigen::Vector3d R1;
        Eigen::Vector3d R2;
        Eigen::Vector3d t;
        SixPointPose() { poses.push_back( Sophus::SE3d() ); }
    };
    
    /** \brief Plane estimation
     *
     * Plane estimation from three points.
     */
    struct ThreePointPlane : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        bool canRefine();
        double score( PointPairList::iterator it );
        Eigen::Vector4d plane;
    };

    /** \brief Upright pose
     *
     * Pose estimation using the upright constraint.
     */
    struct UprightPose : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        double score( PointPairList::iterator it );
        Sophus::SE3d pose;
    };

    /** \brief Homography
     *
     * Homography estimation.
     */
    struct Homography : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        double score( PointPairList::iterator it );
        Eigen::Matrix3d H;
    };
    
    /** \brief Vertical vanishing point
     *
     * Vertical vanishing point estimation.
     */
    struct VerticalVanishingPoint : public Estimator
    {
        int sampleSize();
        int compute( PointPairList::iterator begin, PointPairList::iterator end );
        double score( PointPairList::iterator it );
        Eigen::Vector3d vp;
        Sophus::SO3d R; // R*vp = [0 1 0]'
        bool canRefine();
    };

    /** \brief PROSAC implementation
     *
     * This class implements the PROSAC algorithm for robust estimation.
     */
    struct PROSAC {
        int num_trials;
        double inlier_threshold;
        int min_num_inliers;
        
        PROSAC();
        int compute( PointPairList::iterator begin, PointPairList::iterator end, Estimator &estimator, std::vector<bool> &inliers );
        // can use a restricted set for RANSAC hypotheses, expanded set for finding inliers
        int compute( PointPairList::iterator begin, PointPairList::iterator end, PointPairList::iterator all_begin, PointPairList::iterator all_end, Estimator &estimator, std::vector<bool> &inliers );
        int countInliers( PointPairList::iterator begin, PointPairList::iterator end, Estimator &estimator );
        int countInliers( PointPairList::iterator begin, PointPairList::iterator end, Estimator &estimator, std::vector<bool> &inliers );
        int countInliers( PointPairList::iterator begin, PointPairList::iterator end, int step, Estimator &estimator, std::vector<bool> &inliers );
    };
    
    /**
     * @}
     */
}

#endif
