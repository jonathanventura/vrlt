/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: multiview.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef __MULTIVIEW_H
#define __MULTIVIEW_H

#include <vector>
#include <map>
#include <string>

#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>
#include <opencv2/highgui.hpp>

#include <MultiView/pyramid.h>


namespace vrlt {
/** \addtogroup MultiView
 * \brief Classes to manage, store, and load structure from motion data.
 * @{
 */
    
	/**
     * \brief Generic base class
     *
	 * Generic base class for all MultiView objects.
	 */
    struct Element {
        std::string name;
    };
    
	/**
     * \brief Generic list
     *
	 * Generic list of elements, used for all lists.  Use ElementList::iterator to iterate through it.
	 */
    typedef std::map<std::string,Element*> ElementList;

    /**
     * \brief Calibration parameters
     *
     * Represents the calibration parameters of a camera.
     */
    struct Calibration : public Element {
        typedef enum {
            Perspective,
            Spherical,
            Cylindrical
        } Type;
        Type type;
        double focal;
        Eigen::Vector2d center;
        double k1, k2;
        /** Projects a 3D point from object space to image space. */
        Eigen::Vector2d project3( const Eigen::Vector3d &X );
        /** Projects a 2D point form normalized camera coordinates to image space. */
        Eigen::Vector2d project( const Eigen::Vector2d &point );
        /** Applies radial distortion in image space. */
        Eigen::Vector2d distort( const Eigen::Vector2d &point );
        /** Unprojects a 2D point from image space to 3D object space. */
        Eigen::Vector3d unproject( const Eigen::Vector2d &location );
        Calibration() : type( Perspective ), k1( 0 ), k2( 0 ) { }
        
        /** Prepares the cached calibration matrix K. */
        void makeK();
        /** Prepares the cached inverse calibration matrix Kinv. */
        void makeKinverse();
        Eigen::Matrix3f K;
        Eigen::Matrix3f Kinv;
        
        /** Returns the result of X * Kinv. */
        Eigen::Vector3f postMultiplyKinv( const Eigen::Vector3f &X );
    };
    
    struct Reconstruction;
    struct Node;
    /** \brief Camera info
     *
     * Represents the information for a single camera image capture.
     */
    struct Camera : public Element {
        Calibration *calibration;
        std::string path;
        
        // sensor data
        double timestamp;
        double heading;
        Sophus::SO3d attitude;
        Eigen::Matrix<float,6,1> velocity;
        Eigen::Vector3d rotationRate;
        
        cv::Mat color_image;
        cv::Mat image;
        ImagePyramid pyramid;
        cv::Mat float_image;
        cv::Mat small_byte_image;
        cv::Mat small_image;
        ElementList features;
        Node *node;
        Camera() : calibration( NULL ), timestamp( 0 ), velocity( Eigen::Matrix<float,6,1>::Zero() ), rotationRate( Eigen::Vector3d::Zero() ), node( NULL ), isnew( false ) { }
        
        unsigned int texID;
        
        /** Used to mark frames added to the model for tracking */
        bool isnew;
        
        /** Cached matrix for perspective patch projection. */
        Eigen::Matrix3f KAKinv;
        /** Cached matrix for perspective patch projection. */
        Eigen::Vector3f Ka;
    };
    
    template<int d,typename T>
    inline Eigen::Matrix<T,d-1,1> project( const Eigen::Matrix<T,d,1> &Xin )
    {
        return Xin.head(d-1)/Xin[d-1];
    }

    template<int d,typename T>
    inline Eigen::Matrix<T,d+1,1> unproject( const Eigen::Matrix<T,d,1> &Xin )
    {
        Eigen::Matrix<T,d+1,1> Xout;
        Xout.head(d) = Xin;
        Xout[d] = 1;
        return Xout;
    }
    
    Eigen::Vector2d sphericalProject( const Eigen::Vector3d &X );
    Eigen::Vector3d sphericalUnproject( const Eigen::Vector2d &pt );
    
    Eigen::Vector2d cylindricalProject( const Eigen::Vector3d &X );
    Eigen::Vector3d cylindricalUnproject( const Eigen::Vector2d &pt );
    
    struct Track;
    /** \brief An interest point
     *
     * Represents a interest point in an image.
     */
    struct Feature : public Element {
        Track *track;
        Camera *camera;
        Eigen::Vector2d location;
        double orientation;
        double scale;
        unsigned char *descriptor;
        float *floatdescriptor;
        unsigned char color[3];
        unsigned int word;
        ElementList matches;
        Eigen::Vector3d unproject();
        Eigen::Vector3d globalUnproject( Node *root = NULL );
        Feature() : track( NULL ), camera( NULL ), descriptor( NULL ) { }
        ~Feature() { delete [] descriptor; }
    };
    
    /** \brief Feature match
     *
     * A match between two features.
     */
    struct Match : public Element {
        Feature *feature1;
        Feature *feature2;
        double score;
    };
    
    /** \brief Image pair
     *
     * A pair of images with estimated relative pose.
     */
    struct Pair : public Element {
        Node *node1;
        Node *node2;
        Sophus::SE3d pose;
        int nmatches;
        Pair() : node1( NULL ), node2( NULL ) { }
    };
    
    struct Point;
    /** \brief Feature track
     *
     * A track of features across several images.
     * A track can have an associated triangulated point.
     */
    struct Track : public Element {
        ElementList features;
        Point *point;
        Track() : point( NULL ) { }
    };
    
    /** \brief Node in scene graph
     *
     * Camera poses are represented in a hierarchical scene graph.
     * Internal nodes are used to group images.  Leaf nodes should be connected
     * to cameras, so that they represent the pose of the camera.
     * The chain of nodes from leaf to root is used to compute the global pose
     * of a camera.
     *
     * A node represents a reference frame, where the is at the origin.
     * Children of the node are in the reference frame of the parent.
     *
     * A node can contain points which are then stored in the
     * reference frame of the node.
     */
    struct Node : public Element {
        bool fixed;
        bool scaleFixed;
        Node *parent;
        Camera *camera;
        Sophus::SE3d pose;
        Sophus::SE3d precomputedGlobalPose;
        Sophus::SE3d globalPose( Node *root = NULL );
        Sophus::SO3d globalRotation( Node *root = NULL );
        ElementList points;
        ElementList children;
        Node *root();
        Node() : fixed( false ), scaleFixed( false ), parent( NULL ), camera( NULL ) { }
    };
    
    /** \brief 3D triangulated point
     *
     * A 3D triangulated point.
     */
    struct Point : public Element {
        Node *node;
        Track *track;
        Eigen::Vector2f location;
        Eigen::Vector4d position;
        Eigen::Vector3d normal;
        Point() : node( NULL ), track( NULL ), normal( Eigen::Vector3d::Zero() ), cov( Eigen::Matrix2f::Identity() ) { }
        bool tracked;
        int bestLevel;
        Eigen::Matrix2f cov;
        Eigen::Matrix2f invcov;
    };
    
    /** \brief Container for all SfM data
     *
     * This class is used to contain all structure from motion data
     * from a reconstruction.  The class can be written to file and read back
     * for interchange between applications.
     */
    struct Reconstruction {
        ElementList calibrations;
        ElementList cameras;
        ElementList features;
        ElementList matches;
        ElementList triplets;
        ElementList pairs;
        ElementList tracks;
        ElementList nodes;
        
        bool upright;
        std::string pathPrefix;
        std::string featureFileSuffix;
        int utmZone;
        bool utmNorth;
        double utmCenterEast;
        double utmCenterNorth;
        Reconstruction() : upright( false ), pathPrefix( "." ), featureFileSuffix( "key" ), utmZone(0), utmNorth(true), utmCenterEast(0.), utmCenterNorth(0.) { }
        
        void link( Node *node, Point *point );
        void link( Track *track, Point *point );
        void unlink( Track *track, Feature *feature );
        void remove( Track *track );
        void attach( Node *node, Node *root, const Sophus::SE3d &pose );
        void clearPairs( Node *root );
    };
    
    void transformPoints( Node *node, Sophus::SE3d &pose );
    void transformPoints( Node *node, Sophus::Sim3d &transform );
    void removeCameraFeatures( Reconstruction &r, Camera *camera );
    Camera * addCameraToReconstruction( Reconstruction &r, const Calibration *_calibration, const cv::Mat &image, const Sophus::SE3d &pose );
    cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt);
    uchar getGraySubpix(const cv::Mat& img, cv::Point2f pt);

    template<typename T>
    Eigen::Matrix<T,2,1> makeVector( T x, T y )
    {
        Eigen::Matrix<T,2,1> v;
        v << x, y;
        return v;
    }

    template<typename T>
    Eigen::Matrix<T,3,1> makeVector( T x, T y, T z )
    {
        Eigen::Matrix<T,3,1> v;
        v << x, y, z;
        return v;
    }

    template<typename T>
    Eigen::Matrix<T,4,1> makeVector( T x, T y, T z, T w )
    {
        Eigen::Matrix<T,4,1> v;
        v << x, y, z, w;
        return v;
    }

    /** @}
     */
}


#endif
