/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: tracker.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/tracker.h>
#include <PatchTracker/patch.h>
#include <PatchTracker/nccsearch.h>

#include <cvd/thread.h>

#include <cvd/image_io.h>
#include <cvd/draw.h>

#include <TooN/Lapack_Cholesky.h>
#include <TooN/wls.h>

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
    using namespace TooN;
    using namespace CVD;
    using namespace std;
    
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
    
    Tracker::Tracker( Node *_root, int _maxnumpoints, string method, double threshold )
    : root( _root ), ntracked( 0 ), nattempted( 0 ), verbose( false ),
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
        SE3<> poseinv = mynode->pose.inverse();
        SE3<> rel_pose = source->node->precomputedGlobalPose * poseinv;
        source->KAKinv = source->calibration->K * rel_pose.get_rotation().get_matrix() * target->calibration->Kinv;
        source->Ka.T()[0] = source->calibration->K * rel_pose.get_translation();
    }
    
    void Tracker::updateMatrices( Camera *target )
    {
        SE3<> poseinv = mynode->pose.inverse();
        for ( int i = 0; i < cameras.size(); i++ )
        {
            SE3<> rel_pose = cameras[i]->node->precomputedGlobalPose * poseinv;
            cameras[i]->KAKinv = cameras[i]->calibration->K * rel_pose.get_rotation().get_matrix() * target->calibration->Kinv;
            cameras[i]->Ka.T()[0] = cameras[i]->calibration->K * rel_pose.get_translation();
        }
    }
    
    void checkPoint( void *context, size_t i )
    {
        Tracker *tracker = (Tracker *)context;
        
        Point *point = tracker->patches[i]->point;
        point->tracked = false;

        tracker->searchPatches[i] = NULL;

        Vector<4,float> pointData;
        Vector<3,float> PX;
        Vector<2,float> proj;
        Vector<3,float> normalData;
        Vector<3,float> PN;

        // check in front of camera
        pointData = point->position;
#ifdef USE_ACCELERATE
        vDSP_dotpr( tracker->poseMatrix.get_data_ptr() + 8, 1, pointData.get_data_ptr(), 1, PX.get_data_ptr()+2, 4 );
#else
        PX[2] = tracker->poseMatrix[2] * pointData;
#endif
        if ( PX[2] < 0 ) {
            return;
        }
        
        // check in image
#ifdef USE_ACCELERATE
        vDSP_dotpr( tracker->poseMatrix.get_data_ptr()    , 1, pointData.get_data_ptr(), 1, PX.get_data_ptr()  , 4 );
        vDSP_dotpr( tracker->poseMatrix.get_data_ptr() + 4, 1, pointData.get_data_ptr(), 1, PX.get_data_ptr()+1, 4 );
#else
        PX[0] = tracker->poseMatrix[0] * pointData;
        PX[1] = tracker->poseMatrix[1] * pointData;
#endif
        proj[0] = tracker->f * (PX[0]/PX[2]) + tracker->u;
        proj[1] = tracker->f * (PX[1]/PX[2]) + tracker->v;
        ImageRef pos( (int)proj[0], (int)proj[1] );
        if ( !tracker->current_camera->image.in_image( pos ) )
        {
            return;
        }
                
        normalData = point->normal;
#ifdef USE_ACCELERATE
        vDSP_dotpr( tracker->poseMatrix.get_data_ptr()    , 1, normalData.get_data_ptr(), 1, PN.get_data_ptr()  , 3 );
        vDSP_dotpr( tracker->poseMatrix.get_data_ptr() + 4, 1, normalData.get_data_ptr(), 1, PN.get_data_ptr()+1, 3 );
        vDSP_dotpr( tracker->poseMatrix.get_data_ptr() + 8, 1, normalData.get_data_ptr(), 1, PN.get_data_ptr()+2, 3 );
#else
        PN = tracker->poseMatrix.slice<0,0,3,3>() * normalData;
#endif
      
        float D;
#ifdef USE_ACCELERATE
        vDSP_dotpr( PN.get_data_ptr(), 1, PX.get_data_ptr(), 1, &D, 3 );
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
        poseMatrix.slice<0,0,3,3>() = c->node->pose.get_rotation().get_matrix();
        poseMatrix.T()[3] = c->node->pose.get_translation();
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
        
        prev_pose = SE3<>();
        
        vector<Point*> new_points;
        vector<Patch*> new_patches;
        
        // reduce number of patches
        if ( count > maxnumpoints )
        {
            count = maxnumpoints;
        }
        
        nattempted = count;

        int firstcount = count;
        if ( verbose ) cout << "first count: " << count << "\n";
        
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
            sort( searchPatches.begin(), searchPatches.begin()+count, SortPatches() );
            count = newcount;
            if ( verbose ) cout << "after templates: count at level " << level << ": " << count << "\n";

            patchSearcher->begin = searchPatches.begin();
            newcount = patchSearcher->doSearch( count );
            sort( searchPatches.begin(), searchPatches.begin()+count, SortPatches() );
            count = newcount;
            if ( verbose ) cout << "after search: count at level " << level << ": " << count << "\n";
            
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
                prev_features.push_back( sourcepatch->source->calibration->project( project( sourcepatch->point->position.slice<0,3>() ) ) );
                next_features.push_back( sourcepatch->targetPos );
            }
        }
        
        bool tracked = ( nattempted >= minnumpoints ) && ( ntracked >= minratio * nattempted );
        
        return tracked;
    }
}
