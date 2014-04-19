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
    typedef std::pair< TooN::Vector<3>, TooN::Vector<3> > PointPair;
    /** \brief List of 3D point pairs */
    typedef std::vector<PointPair> PointPairList;

    /** Computes the reprojection error of a feature in its camera image. */
    TooN::Vector<2> computeReprojError( Feature *feature );
    /** Triangulates a point given a relative pose and a pair of homogeneous observations. */
    TooN::Vector<4> triangulate( const TooN::SE3<> &pose, PointPair point_pair );
    /** Triangulates a point given two features. */
    TooN::Vector<4> triangulate( Feature *feature1, Feature *feature2 );
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
        std::vector< TooN::Matrix<3> > Elist;
        TooN::Matrix<3> E;
        void chooseSolution( int soln );
        TooN::SE3<> getPose( PointPairList::iterator begin, PointPairList::iterator end );
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
        TooN::Matrix<3> E;
        TooN::SE3<> getPose( PointPairList::iterator begin, PointPairList::iterator end );
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
        std::vector< TooN::SE3<> > poses;
        TooN::SE3<> pose;
        TooN::Vector<3> R0;
        TooN::Vector<3> R1;
        TooN::Vector<3> R2;
        TooN::Vector<3> t;
        bool canRefine();
        ThreePointPose() { poses.push_back( TooN::SE3<>() ); }
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
        std::vector< TooN::SE3<> > poses;
        TooN::SE3<> pose;
        TooN::Vector<3> R0;
        TooN::Vector<3> R1;
        TooN::Vector<3> R2;
        TooN::Vector<3> t;
        SixPointPose() { poses.push_back( TooN::SE3<>() ); }
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
        TooN::Vector<4> plane;
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
        TooN::SE3<> pose;
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
        TooN::Matrix<3> H;
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
        TooN::Vector<3> vp;
        TooN::SO3<> R; // R*vp = [0 1 0]'
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
