/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: tracker.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/tracker.h>
#include <PatchTracker/patch.h>
#include <PatchTracker/nccsearch.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Eigen>

#include <iostream>

#ifdef USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#endif

#ifdef USE_DISPATCH
#include <dispatch/dispatch.h>
#endif

#ifdef __APPLE__
#include "TargetConditionals.h"
#endif

namespace vrlt
{
    static inline bool in_image( const cv::Size &size, const cv::Point2i &loc )
    {
        return ( loc.x >= 0 && loc.x < size.width && loc.y >= 0 && loc.y < size.height );
    }

    struct SortPatches
    {
        bool operator()( const Patch *a, const Patch *b ) { return ( a->shouldTrack > b->shouldTrack ); }
    };
    
    void Tracker::precomputeGlobalPoses( Node *node )
    {
        if ( node->camera != NULL ) cameras.push_back( node->camera );
        node->precomputedGlobalPose = node->globalPose();
        if ( node->camera != NULL ) node->camera->calibration->makeK();
        ElementList::iterator it;
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node*)it->second;
            precomputeGlobalPoses( child );
        }   
    }
    
    Tracker::Tracker( Node *_root, int _maxnumpoints, std::string method, double threshold )
    : ntracked( 0 ), root( _root ), nattempted( 0 ), verbose( false ),
    maxnumpoints( _maxnumpoints ), firstlevel( 3 ), lastlevel( 1 ), niter( 10 ),
    recompute_sigmasq(true)
    {
        do_pose_update = true;
        
        precomputeGlobalPoses( root );
        
        cameras[0]->calibration->makeK();
        
        patches.resize( root->points.size() );
        visiblePatches.resize( root->points.size() );
        searchPatches.resize( root->points.size() );
        
        int i = 0;
        ElementList::iterator it;
        for ( it = root->points.begin(); it != root->points.end(); it++,i++ )
        {
            Point *point = (Point *)it->second;
            patches[i] = new Patch( point );
        }
        random_shuffle( patches.begin(), patches.end() );
        patchSearcher = NULL;
		patchSearcher = new PatchSearchNCC( maxnumpoints );
		patchSearcher->lowThreshold = threshold;
		patchSearcher->highThreshold = threshold;
        
        mynode = new Node;
        mycamera = new Camera;
        mycalibration = new Calibration;
        
        mycamera->name = "camera";
        mycamera->calibration = mycalibration;
        mycamera->node = mynode;
        
        mynode->name = "node";
        mynode->camera = mycamera;
    }
    
    Tracker::~Tracker()
    {
        ElementList::iterator it;
        int i = 0;
        for ( it = root->points.begin(); it != root->points.end(); it++,i++ )
        {
            delete patches[i];
        }
        delete patchSearcher;
        
        delete mynode;
        delete mycamera;
        delete mycalibration;
    }
    
    void Tracker::updateMatrices( Camera *source, Camera *target )
    {
        Sophus::SE3d poseinv = mynode->pose.inverse();
        Sophus::SE3d rel_pose = source->node->precomputedGlobalPose * poseinv;
        source->KAKinv = source->calibration->K * rel_pose.so3().matrix().cast<float>() * target->calibration->Kinv;
        source->Ka.col(0) = source->calibration->K * rel_pose.translation().cast<float>();
    }
    
    void Tracker::updateMatrices( Camera *target )
    {
        Sophus::SE3d poseinv = mynode->pose.inverse();
        for ( int i = 0; i < cameras.size(); i++ )
        {
            Sophus::SE3d rel_pose = cameras[i]->node->precomputedGlobalPose * poseinv;
            cameras[i]->KAKinv = cameras[i]->calibration->K * rel_pose.so3().matrix().cast<float>() * target->calibration->Kinv;
            cameras[i]->Ka.col(0) = cameras[i]->calibration->K * rel_pose.translation().cast<float>();
        }
    }
    
    void checkPoint( void *context, size_t i )
    {
        Tracker *tracker = (Tracker *)context;
        
        Point *point = tracker->patches[i]->point;
        point->tracked = false;

        tracker->searchPatches[i] = NULL;

        Eigen::Vector4f pointData;
        Eigen::Vector3f PX;
        Eigen::Vector2f proj;
        Eigen::Vector3f normalData;
        Eigen::Vector3f PN;

        // NB: Eigen is column major
        // check in front of camera
        pointData = point->position.cast<float>();
#ifdef USE_ACCELERATE
        vDSP_dotpr( tracker->poseMatrix.data() + 3, 4, pointData.data(), 1, PX.data()+2, 4 );
#else
        PX[2] = tracker->poseMatrix.row(2).dot( pointData );
#endif
        if ( PX[2] < 0 ) {
            return;
        }
        
        // check in image
#ifdef USE_ACCELERATE
        vDSP_dotpr( tracker->poseMatrix.data()    , 4, pointData.data(), 1, PX.data()  , 4 );
        vDSP_dotpr( tracker->poseMatrix.data() + 1, 4, pointData.data(), 1, PX.data()+1, 4 );
#else
        PX[0] = tracker->poseMatrix.row(0).dot( pointData );
        PX[1] = tracker->poseMatrix.row(1).dot( pointData );
#endif
        proj[0] = tracker->f * (PX[0]/PX[2]) + tracker->u;
        proj[1] = tracker->f * (PX[1]/PX[2]) + tracker->v;
        cv::Point2i pos( (int)proj[0], (int)proj[1] );
        if ( !in_image( tracker->current_camera->image.size(), pos ) )
        {
            return;
        }
                
        normalData = point->normal.cast<float>();
#ifdef USE_ACCELERATE
        vDSP_dotpr( tracker->poseMatrix.data()    , 4, normalData.data(), 1, PN.data()  , 3 );
        vDSP_dotpr( tracker->poseMatrix.data() + 1, 4, normalData.data(), 1, PN.data()+1, 3 );
        vDSP_dotpr( tracker->poseMatrix.data() + 2, 4, normalData.data(), 1, PN.data()+2, 3 );
#else
        PN = tracker->poseMatrix.block<3,3>(0,0) * normalData;
#endif
      
        float D;
#ifdef USE_ACCELERATE
        vDSP_dotpr( PN.data(), 1, PX.data(), 1, &D, 3 );
        D = -D;
#else
        D = -(PN * PX);
#endif
        
        // choose source image
        Patch *sourcepatch = tracker->patches[i];
        sourcepatch->shouldTrack = true;
        sourcepatch->bestLevel = 4;
        sourcepatch->PX = PX;
        sourcepatch->PN = PN;
        sourcepatch->D = D;
        sourcepatch->center = proj;
        sourcepatch->warpcenter = sourcepatch->center;
        sourcepatch->source = NULL;
        sourcepatch->updateTarget( tracker->current_camera );
        sourcepatch->chooseSource( tracker->use_new_cameras );
        if ( sourcepatch->source == NULL ) {
            return;
        }
        
        tracker->visiblePatches[i] = sourcepatch;
    }
    
    void preparePatch( void *context, size_t i )
    {
        Tracker *tracker = (Tracker*)context;
        
        Patch *sourcepatch = tracker->searchPatches[i];
        
        sourcepatch->center[0] = tracker->f * (sourcepatch->PX[0]/sourcepatch->PX[2]) + tracker->u;
        sourcepatch->center[1] = tracker->f * (sourcepatch->PX[1]/sourcepatch->PX[2]) + tracker->v;
        sourcepatch->warpcenter = sourcepatch->center;
        sourcepatch->updateTarget( tracker->current_camera );
    }
        
    void levelupPatch( void *context, size_t i )
    {
        Tracker *tracker = (Tracker*)context;
        
        Patch *sourcepatch = tracker->searchPatches[i];
        
        sourcepatch->center[0] = ( sourcepatch->targetPos[0] + .5f ) * 2.f - .5f;
        sourcepatch->center[1] = ( sourcepatch->targetPos[1] + .5f ) * 2.f - .5f;
        sourcepatch->warpcenter[0] = ( sourcepatch->warpcenter[0] + .5f ) * 2.f - .5f;
        sourcepatch->warpcenter[1] = ( sourcepatch->warpcenter[1] + .5f ) * 2.f - .5f;
        sourcepatch->updateTarget( tracker->current_camera );
    }
    
    void Tracker::setCurrentCamera( Camera *c )
    {
        current_camera = c;
        updateMatrices( c );
        poseMatrix.block<3,3>(0,0) = c->node->pose.so3().matrix().cast<float>();
        poseMatrix.col(3) = c->node->pose.translation().cast<float>();
        f = c->calibration->focal;
        u = c->calibration->center[0];
        v = c->calibration->center[1];
    }
    
    bool Tracker::track( Camera *camera_in, bool _use_new_cameras )
    {
        use_new_cameras = _use_new_cameras;
        
        mynode->pose = camera_in->node->pose;
        
#ifdef USE_DISPATCH
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
#endif
        
        camera_in->calibration->makeKinverse();
        setCurrentCamera( camera_in );
        
        for ( size_t i = 0; i < visiblePatches.size(); i++ ) visiblePatches[i] = NULL;
        
        // find all points which are potentially visible in the image
#ifdef USE_DISPATCH
        dispatch_apply_f( patches.size(), queue, this, checkPoint );
#else
        for ( size_t i = 0; i < patches.size(); i++ ) checkPoint( this, i );
#endif
        
        int count = 0;
        for ( int i = 0; i < visiblePatches.size(); i++ )
        {
            if ( visiblePatches[i] != NULL ) {
                searchPatches[count++] = visiblePatches[i];
            }
        }
        
        prev_pose = Sophus::SE3d();
        
        std::vector<Point*> new_points;
        std::vector<Patch*> new_patches;
        
        // reduce number of patches
        if ( count > maxnumpoints )
        {
            count = maxnumpoints;
        }
        
        nattempted = count;

        int firstcount = count;
        if ( verbose ) std::cout << "first count: " << count << "\n";
        
        // iterate through pyramid
        for ( int level = firstlevel; level >= lastlevel; level-- )
        {
            float levelScale = powf( 2.f, -level );
            patchSearcher->level = level;

            // set up camera calibration and image
            mycalibration->focal = camera_in->calibration->focal * levelScale;
            mycalibration->center[0] = ( camera_in->calibration->center[0] + .5f ) * levelScale - .5f;
            mycalibration->center[1] = ( camera_in->calibration->center[1] + .5f ) * levelScale - .5f;
            mycalibration->makeKinverse();
            mycamera->image = camera_in->pyramid.levels[level].image;

            setCurrentCamera( mycamera );
            if ( level == firstlevel ) {
#ifdef USE_DISPATCH
                dispatch_apply_f( count, queue, this, preparePatch );
#else
                for ( int i = 0; i < count; i++ ) preparePatch( this, i );
#endif
            } else {
#ifdef USE_DISPATCH
                dispatch_apply_f( count, queue, this, levelupPatch );
#else
                for ( int i = 0; i < count; i++ ) levelupPatch( this, i );
#endif
            }
            
            int newcount;
            patchSearcher->begin = searchPatches.begin();
            newcount = patchSearcher->makeTemplates( count );
            std::sort( searchPatches.begin(), searchPatches.begin()+count, SortPatches() );
            count = newcount;
            if ( verbose ) std::cout << "after templates: count at level " << level << ": " << count << "\n";

            patchSearcher->begin = searchPatches.begin();
            newcount = patchSearcher->doSearch( count );
            std::sort( searchPatches.begin(), searchPatches.begin()+count, SortPatches() );
            count = newcount;
            if ( verbose ) std::cout << "after search: count at level " << level << ": " << count << "\n";
            
            if ( level == firstlevel )
            {
                ntracked = 0;
                nnew = 0;
                ntracked = count;
                for ( int i = 0; i < count; i++ ) {
                    searchPatches[i]->point->tracked = true;
                    if ( searchPatches[i]->source->isnew ) nnew++;
                }
                
                bool tracked = ( nattempted >= minnumpoints ) && ( ntracked >= minratio * nattempted );
                if ( !tracked ) return false;
            }
        }
        camera_in->node->pose = current_camera->node->pose;
        
        // set location of patches to zero level
        prev_features.clear();
        next_features.clear();
        for ( int i = 0; i < firstcount; i++ )
        {
            Patch *sourcepatch = searchPatches[i];
            if ( !sourcepatch->point->tracked ) continue;
            sourcepatch->point->bestLevel = sourcepatch->bestLevel;
            float scale = 1 << sourcepatch->bestLevel;
            sourcepatch->targetPos[0] = ( sourcepatch->targetPos[0] + .5f ) * scale - .5f;
            sourcepatch->targetPos[1] = ( sourcepatch->targetPos[1] + .5f ) * scale - .5f;
            sourcepatch->point->location = sourcepatch->targetPos;
            
            if ( sourcepatch->point->position[3] == 0 ) {
                Eigen::Vector3d X = sourcepatch->point->position.head(3);
                prev_features.push_back( sourcepatch->source->calibration->project( project( X ) ) );
                next_features.push_back( sourcepatch->targetPos.cast<double>() );
            }
        }
        
        bool tracked = ( nattempted >= minnumpoints ) && ( ntracked >= minratio * nattempted );
        
        return tracked;
    }
}
