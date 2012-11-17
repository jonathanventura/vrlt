/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
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

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <MultiView/pyramid.h>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

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
        TooN::Vector<2> center;
        double k1, k2;
        /** Projects a 3D point from object space to image space. */
        TooN::Vector<2> project3( const TooN::Vector<3> &X );
        /** Projects a 2D point form normalized camera coordinates to image space. */
        TooN::Vector<2> project( const TooN::Vector<2> &point );
        /** Applies radial distortion in image space. */
        TooN::Vector<2> distort( const TooN::Vector<2> &point );
        /** Unprojects a 2D point from image space to 3D object space. */
        TooN::Vector<3> unproject( const TooN::Vector<2> &location );
        Calibration() : type( Perspective ), k1( 0 ), k2( 0 ) { }
        
        /** Prepares the cached calibration matrix K. */
        void makeK();
        /** Prepares the cached inverse calibration matrix Kinv. */
        void makeKinverse();
        TooN::Matrix<3,3,float> K;
        TooN::Matrix<3,3,float> Kinv;
        
        /** Returns the result of X * Kinv. */
        TooN::Vector<3,float> postMultiplyKinv( const TooN::Vector<3,float> &X );
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
        TooN::SO3<> attitude;
        TooN::Vector<6,float> velocity;
        TooN::Vector<3> rotationRate;
        
        CVD::Image< CVD::Rgb<CVD::byte> > color_image;
        CVD::Image<CVD::byte> image;
        ImagePyramid pyramid;
        CVD::Image<float> float_image;
        CVD::Image<CVD::byte> small_byte_image;
        CVD::Image<float> small_image;
        ElementList features;
        Node *node;
        Camera() : calibration( NULL ), node( NULL ), velocity( TooN::Zeros ), rotationRate( TooN::Zeros ), timestamp( 0 ), isnew( false ) { }
        
        unsigned int texID;
        
        /** Used to mark frames added to the model for tracking */
        bool isnew;
        
        /** Cached matrix for perspective patch projection. */
        TooN::Matrix<3,3,float> KAKinv;
        /** Cached matrix for perspective patch projection. */
        TooN::Matrix<3,1,float> Ka;
    };
    
    TooN::Vector<2> sphericalProject( const TooN::Vector<3> &X );
    TooN::Vector<3> sphericalUnproject( const TooN::Vector<2> &pt );
    TooN::Vector<2> cylindricalProject( const TooN::Vector<3> &X );
    TooN::Vector<3> cylindricalUnproject( const TooN::Vector<2> &pt );

    struct Track;
    /** \brief An interest point
     *
     * Represents a interest point in an image.
     */
    struct Feature : public Element {
        Track *track;
        Camera *camera;
        TooN::Vector<2> location;
        double orientation;
        double scale;
        unsigned char *descriptor;
        float *floatdescriptor;
        unsigned char color[3];
        unsigned int word;
        ElementList matches;
        TooN::Vector<3> unproject();
        TooN::Vector<3> globalUnproject( Node *root = NULL );
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
        TooN::SE3<> pose;
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
        TooN::SE3<> pose;
        TooN::SE3<> precomputedGlobalPose;
        TooN::SE3<> globalPose( Node *root = NULL );
        TooN::SO3<> globalRotation( Node *root = NULL );
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
        TooN::Vector<2,float> location;
        TooN::Vector<4> position;
        TooN::Vector<3> normal;
        Point() : node( NULL ), track( NULL ), normal( TooN::Zeros ), cov( TooN::Identity ) { }
        bool tracked;
        int bestLevel;
        TooN::Matrix<2,2,float> cov;
        TooN::Matrix<2,2,float> invcov;
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
        Reconstruction() : upright( false ), pathPrefix( "." ), featureFileSuffix( "key" ) { }
        
        void link( Node *node, Point *point );
        void link( Track *track, Point *point );
        void unlink( Track *track, Feature *feature );
        void remove( Track *track );
        void attach( Node *node, Node *root, const TooN::SE3<> &pose );
        void clearPairs( Node *root );
    };
    
    void removeCameraFeatures( Reconstruction &r, Camera *camera );
    Camera * addCameraToReconstruction( Reconstruction &r, const Calibration *_calibration, const CVD::BasicImage<CVD::byte> &image, const TooN::SE3<> &pose );
    
    /** @}
     */
}


#endif
