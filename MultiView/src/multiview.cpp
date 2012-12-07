/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: multiview.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <MultiView/multiview.h>

namespace vrlt
{
    using namespace std;
    using namespace TooN;
    
    Vector<3> Feature::unproject()
    {
        return camera->calibration->unproject( location );
    }
    
    Vector<3> Feature::globalUnproject( Node *root )
    {
        return this->camera->node->globalRotation( root ).inverse() * this->unproject();
    }
    
    Vector<2> Calibration::project3( const TooN::Vector<3> &X )
    {
        if ( type == Spherical ) return this->project( sphericalProject( X ) );
        else if ( type == Cylindrical ) return this->project( cylindricalProject( X ) );
        return this->project( TooN::project( X ) );
    }
    
    Vector<2> Calibration::project( const TooN::Vector<2> &point )
    {
        Vector<2> location;
        location[0] = focal * point[0] + center[0];
        location[1] = focal * point[1] + center[1];
        return location;
    }
    
    Vector<2> Calibration::distort( const TooN::Vector<2> &point )
    {
        float r = point*point;
        float rsq = r * r;
        float s = (1. + k1 * rsq + k2 * rsq * rsq );
        return s * point;
    }
    
    Vector<3> Calibration::unproject( const Vector<2> &location )
    {
        Vector<2> point;
        point[0] = ( location[0] - center[0] ) / focal;
        point[1] = ( location[1] - center[1] ) / focal;
        if ( type == Spherical ) return sphericalUnproject( point );
        if ( type == Cylindrical ) return cylindricalUnproject( point );
        return TooN::unproject( point );
    }
    
    void Calibration::makeK()
    {
        K = Identity;
        K(0,0) = K(1,1) = focal;
        K(0,2) = center[0];
        K(1,2) = center[1];
    }
        
    void Calibration::makeKinverse()
    {
        Kinv = Identity;
        float invf = 1.f / focal;
        Kinv(0,0) = Kinv(1,1) = invf;
        Kinv(0,2) = -center[0] * invf;
        Kinv(1,2) = -center[1] * invf;
    }
    
    Vector<3,float> Calibration::postMultiplyKinv( const Vector<3,float> &X )
    {
        Vector<3,float> out;
        
        out[0] = Kinv(0,0) * X[0];
        out[1] = Kinv(1,1) * X[1];
        out[2] = Kinv(0,2) * X[0] + Kinv(1,2) * X[1] + X[2];
        
        return out;
    }
    
    Vector<2> sphericalProject( const TooN::Vector<3> &X )
    {
        double theta = atan2( X[0], X[2] );
        double phi = asin( X[1] / norm(X) );
        
        return makeVector( theta, phi );
    }
    
    Vector<3> sphericalUnproject( const TooN::Vector<2> &pt )
    {
        double theta = pt[0];
        double phi = pt[1];
        
        Vector<3> X;
        X[0] = sin( theta ) * cos( phi );
        X[1] = sin( phi );
        X[2] = cos( theta ) * cos( phi );
        
        return X;
    }
    
    Vector<2> cylindricalProject( const TooN::Vector<3> &X )
    {
        double theta = atan2( X[0], X[2] );
        double h = X[1] / sqrt( X[0] * X[0] + X[2] * X[2] );
        
        return makeVector( theta, h );
    }
    
    Vector<3> cylindricalUnproject( const TooN::Vector<2> &pt )
    {
        double theta = pt[0];
        double h = pt[1];
        
        Vector<3> X;
        X[0] = sin( theta );
        X[1] = h;
        X[2] = cos( theta );
        
        return X;
    }
    
    Node* Node::root()
    {
        Node *myroot = this;
        while ( myroot->parent != NULL ) myroot = myroot->parent;
        return myroot;
    }
    
    SE3<> Node::globalPose( Node *root )
    {
        SE3<> globalpose;
        Node *node = this;
        while ( node != root ) {
            globalpose = globalpose * node->pose;
            node = node->parent;
        }
        return globalpose;
    }
    
    SO3<> Node::globalRotation( Node *root )
    {
        SO3<> globalrot;
        Node *node = this;
        while ( node != root ) {
            globalrot = globalrot * node->pose.get_rotation();
            node = node->parent;
        }
        return globalrot;
    }
    
    void Reconstruction::link( Node *node, Point *point )
    {
        node->points[point->name] = point;
        point->node = node;
    }
    
    void Reconstruction::link( Track *track, Point *point )
    {
        track->point = point;
        point->track = track;
    }
    
    void Reconstruction::unlink( Track *track, Feature *feature )
    {
        track->features.erase( feature->name );
        feature->track = NULL;
    }
    
    void Reconstruction::remove( Track *track )
    {
        // remove feature links
//        ElementList::iterator it;
//        for ( it = track->features.begin(); it != track->features.end(); it++ )
//        {
//            Feature *feature = (Feature*)it->second;
//            feature->track = NULL;
//        }
        
        // remove point
        Point *point = track->point;
        if ( point != NULL )
        {
            Node *node = point->node;
            if ( node != NULL ) node->points.erase( point->name );
            delete point;
        }
        track->point = NULL;
        
        // remove track
//        tracks.erase( track->name );
//        delete track;
    }
    
    void Reconstruction::attach( Node *node, Node *root, const TooN::SE3<> &pose )
    {
        // get this camera's root node
        Node *oldroot = node->root();
        SE3<> oldpose = node->globalPose();
        
        // remove from top level list if necessary
        if ( nodes.count( oldroot->name ) > 0 ) {
            nodes.erase( oldroot->name );
        }
        
        // attach to new root
        root->children[ oldroot->name ] = oldroot;
        oldroot->parent = root;
        
        // update pose
        oldroot->pose = oldpose * pose;
    }

    void Reconstruction::clearPairs( Node *root )
    {
        vector<Pair*> pairsToRemove;
        ElementList::iterator it;
        for ( it = pairs.begin(); it != pairs.end(); it++ )
        {
            Pair *pair = (Pair*)it->second;
            if ( pair->node1->root() == root || pair->node2->root() == root )
                pairsToRemove.push_back( pair );
        }
        for ( int i = 0; i < pairsToRemove.size(); i++ )
        {
            pairs.erase( pairsToRemove[i]->name );
            delete pairsToRemove[i];
        }
    }
    
    void removeCameraFeatures( Reconstruction &r, Camera *camera )
    {
        ElementList::iterator it;
        
        // remove features from tracks and delete
        for ( it = camera->features.begin(); it != camera->features.end(); it++ )
        {
            Feature *feature = (Feature*)it->second;
            Track *track = feature->track;
            
            if ( track != NULL ) {
                track->features.erase( feature->name );
            }
            
            r.features.erase( feature->name );
            
            delete feature;
        }
        camera->features.clear();
    }
    
    Camera * addCameraToReconstruction( Reconstruction &r, const Calibration *_calibration, const CVD::BasicImage<CVD::byte> &image, const SE3<> &pose )
    {
        char name[1024];
        
        // make camera
        Camera *camera = new Camera;
        
        // make camera name
        int num_cameras = r.cameras.size();
        sprintf( name, "newcamera%06d", num_cameras+1 );
        camera->name = name;
        camera->isnew = true;
        
        // add camera
        r.cameras[ camera->name ] = camera;
        // * this needs to be done *
        //tracker.cameras.push_back( camera );
        
        // copy image
        camera->image.resize( image.size() );
        camera->image.copy_from( image );
        camera->pyramid.resize( image.size() );
        camera->pyramid.copy_from( image );
        
        // create calibration if necessary
        Calibration *calibration = NULL;
        if ( r.calibrations.count( _calibration->name ) == 0 ) {
            calibration = new Calibration;
            calibration->name = _calibration->name;
            calibration->focal = _calibration->focal;
            calibration->center = _calibration->center;
            calibration->makeK();
            calibration->makeKinverse();
            r.calibrations[ calibration->name ] = calibration;
        } else {
            calibration = (Calibration*)r.calibrations[ _calibration->name ];
        }
        camera->calibration = calibration;
        
        // create node
        Node *node = new Node;
        node->camera = camera;
        camera->node = node;
        
        // make node name
        // add node to root
        Node *root = (Node*)r.nodes["root"];
        int num_nodes = root->children.size();
        sprintf( name, "newnode%06d", num_nodes+1 );
        node->name = name;
        root->children[ node->name ] = node;
        
        // set node pose
        node->pose = pose;
        node->precomputedGlobalPose = pose;
        
        // create and link features for tracked points
        ElementList fixedPoints = root->points;
        int num_features = r.features.size();
        ElementList::iterator it;
        for ( it = root->points.begin(); it != root->points.end(); it++ )
        {
            Point *point = (Point *)it->second;
            Track *track = point->track;
            
            if ( point->tracked ) {
                // make feature name
                Feature *feature = new Feature;
                sprintf( name, "feature%06d", num_features++ );
                feature->name = name;
                
                // add feature
                r.features[ feature->name ] = feature;
                
                // set feature location
                feature->location = point->location;
                
                // set feature color
                feature->color[0] = 255;
                feature->color[1] = 255;
                feature->color[2] = 0;

                // add feature to camera
                feature->camera = camera;
                camera->features[ feature->name ] = feature;
                
                // add feature to track
                track->features[ feature->name ] = feature;
                feature->track = track;
                
                fixedPoints.erase( point->name );
            }
        }

        return camera;
    }
    
}
