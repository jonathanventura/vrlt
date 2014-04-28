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
    Eigen::Vector3d Feature::unproject()
    {
        return camera->calibration->unproject( location );
    }
    
    Eigen::Vector3d Feature::globalUnproject( Node *root )
    {
        return this->camera->node->globalRotation( root ).inverse() * this->unproject();
    }
    
    Eigen::Vector2d Calibration::project3( const Eigen::Vector3d &X )
    {
        if ( type == Spherical ) return this->project( sphericalProject( X ) );
        else if ( type == Cylindrical ) return this->project( cylindricalProject( X ) );
        Eigen::Vector2d x = vrlt::project(X);
        return this->project( x );
    }
    
    Eigen::Vector2d Calibration::project( const Eigen::Vector2d &point )
    {
        Eigen::Vector2d location;
        location[0] = focal * point[0] + center[0];
        location[1] = focal * point[1] + center[1];
        return location;
    }
    
    Eigen::Vector2d Calibration::distort( const Eigen::Vector2d &point )
    {
        float r = point.dot(point);
        float rsq = r * r;
        float s = (1. + k1 * rsq + k2 * rsq * rsq );
        return s * point;
    }
    
    Eigen::Vector3d Calibration::unproject( const Eigen::Vector2d &location )
    {
        Eigen::Vector2d point;
        point[0] = ( location[0] - center[0] ) / focal;
        point[1] = ( location[1] - center[1] ) / focal;
        if ( type == Spherical ) return sphericalUnproject( point );
        if ( type == Cylindrical ) return cylindricalUnproject( point );
        return vrlt::unproject( point );
    }
    
    void Calibration::makeK()
    {
        K = Eigen::Matrix3f::Identity();
        K(0,0) = K(1,1) = focal;
        K(0,2) = center[0];
        K(1,2) = center[1];
    }
        
    void Calibration::makeKinverse()
    {
        Kinv = Eigen::Matrix3f::Identity();
        float invf = 1.f / focal;
        Kinv(0,0) = Kinv(1,1) = invf;
        Kinv(0,2) = -center[0] * invf;
        Kinv(1,2) = -center[1] * invf;
    }
    
    Eigen::Vector3f Calibration::postMultiplyKinv( const Eigen::Vector3f &X )
    {
        Eigen::Vector3f out;
        
        out[0] = Kinv(0,0) * X[0];
        out[1] = Kinv(1,1) * X[1];
        out[2] = Kinv(0,2) * X[0] + Kinv(1,2) * X[1] + X[2];
        
        return out;
    }
    
    Eigen::Vector2d sphericalProject( const Eigen::Vector3d &X )
    {
        double theta = atan2( X[0], X[2] );
        double phi = asin( X[1] / X.norm() );
        
        Eigen::Vector2d x;
        x << theta, phi;
        return x;
    }
    
    Eigen::Vector3d sphericalUnproject( const Eigen::Vector2d &pt )
    {
        double theta = pt[0];
        double phi = pt[1];
        
        Eigen::Vector3d X;
        X[0] = sin( theta ) * cos( phi );
        X[1] = sin( phi );
        X[2] = cos( theta ) * cos( phi );
        
        return X;
    }
    
    Eigen::Vector2d cylindricalProject( const Eigen::Vector3d &X )
    {
        double theta = atan2( X[0], X[2] );
        double h = X[1] / sqrt( X[0] * X[0] + X[2] * X[2] );
        
        Eigen::Vector2d x;
        x << theta, h;
        return x;
    }
    
    Eigen::Vector3d cylindricalUnproject( const Eigen::Vector2d &pt )
    {
        double theta = pt[0];
        double h = pt[1];
        
        Eigen::Vector3d X;
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
    
    Sophus::SE3d Node::globalPose( Node *root )
    {
        Sophus::SE3d globalpose;
        Node *node = this;
        while ( node != root ) {
            globalpose = globalpose * node->pose;
            node = node->parent;
        }
        return globalpose;
    }
    
    Sophus::SO3d Node::globalRotation( Node *root )
    {
        Sophus::SO3d globalrot;
        Node *node = this;
        while ( node != root ) {
            globalrot = globalrot * node->pose.so3();
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
    
    void Reconstruction::attach( Node *node, Node *root, const Sophus::SE3d &pose )
    {
        // get this camera's root node
        Node *oldroot = node->root();
        Sophus::SE3d oldpose = node->globalPose();
        
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
        std::vector<Pair*> pairsToRemove;
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
    
    void transformPoints( Node *node, Sophus::SE3d &pose )
    {
        for ( ElementList::iterator it = node->points.begin(); it != node->points.end(); it++ )
        {
            Point *point = (Point*)it->second;
            
            point->position.head(3) = pose * ( point->position.head(3) / point->position[3] );
            point->position[3] = 1.;
        }
        
        for ( ElementList::iterator it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node*)it->second;
            
            child->pose = child->pose * pose.inverse();
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
    
    Camera * addCameraToReconstruction( Reconstruction &r, const Calibration *_calibration, const cv::Mat &image, const Sophus::SE3d &pose )
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
        image.copyTo( camera->image );
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
                feature->location[0] = (double)point->location[0];
                feature->location[1] = (double)point->location[1];
                
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
    
    cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 3);
        
        int x = (int)pt.x;
        int y = (int)pt.y;
        
        int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
        int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
        int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
        int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);
        
        float a = pt.x - (float)x;
        float c = pt.y - (float)y;
        
        uchar b = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[0] * a) * (1.f - c)
                                 + (img.at<cv::Vec3b>(y1, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[0] * a) * c);
        uchar g = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[1] * a) * (1.f - c)
                                 + (img.at<cv::Vec3b>(y1, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[1] * a) * c);
        uchar r = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[2] * a) * (1.f - c)
                                 + (img.at<cv::Vec3b>(y1, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[2] * a) * c);
        
        return cv::Vec3b(b, g, r);
    }
}
