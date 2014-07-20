/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: nccsearch.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/nccsearch.h>
#include <PatchTracker/ncc.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Eigen>

#ifdef USE_DISPATCH
#include <dispatch/dispatch.h>
#endif

namespace vrlt
{
    static inline bool in_image( const cv::Size &size, const cv::Point2i &loc )
    {
        return ( loc.x >= 0 && loc.x < size.width && loc.y >= 0 && loc.y < size.height );
    }
    
    PatchSearchNCC::PatchSearchNCC( int maxnumpoints )
    {
        lowThreshold = highThreshold = 0.7;
        templates = new uchar[8 * 8 * maxnumpoints];
        targets = new uchar[8 * 8 * maxnumpoints];
        templateA = new float[maxnumpoints];
        templateC = new float[maxnumpoints];
    }
    
    PatchSearchNCC::~PatchSearchNCC()
    {
        delete [] templates;
        delete [] targets;
        delete [] templateA;
        delete [] templateC;
    }
    
    static void makeNCCTemplate( void *context, size_t i )
    {
        PatchSearchNCC *searcher = (PatchSearchNCC*)context;
        Patch *patch = *(searcher->begin+i);
        
        uchar *templatePtr = searcher->templates + i*64;
        float *Aptr = searcher->templateA + i;
        float *Cptr = searcher->templateC + i;
        
        cv::Mat templatePatch( cv::Size(8,8), CV_8UC1, templatePtr );
        
        int level = 0;
        float myscale = patch->scale;
        while ( myscale > 3. ) {
            if ( level == NLEVELS-1 ) break;
            level++;
            myscale /= 4.;
        }
        if ( myscale > 3. || myscale < 0.25 ) {
            patch->shouldTrack = false;
            return;
        }
        float levelScale = powf( 2.f, -level );
        
        bool good = patch->sampler.samplePatch( patch->source->pyramid.levels[level].image, patch->warpcenter, patch->warp, levelScale, templatePatch );
        
        if ( !good ) {
            patch->shouldTrack = false;
            return;
        }
        
        patch->index = i;
        
        unsigned int A = computeSum( templatePtr );
        unsigned int B = computeSumSq( templatePtr );
        
        float Af = A;
        float Bf = B;
        
        float Cf = 1.f / sqrtf( 64 * Bf - Af * Af );
        *Aptr = Af;
        *Cptr = Cf;
    }
    
    int PatchSearchNCC::makeTemplates( int count )
    {
#ifdef USE_DISPATCH
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
        dispatch_apply_f( count, queue, this, makeNCCTemplate );
#else
        for ( int i = 0; i < count; i++ ) {
            warpTimer.start();
            makeNCCTemplate( this, i );
            warpTimer.stop();
        }
#endif
        
        int newcount = 0;
        for ( int i = 0; i < count; i++ )
        {
            Patch *patch = *(begin+i);
            if ( patch->shouldTrack ) newcount++;
        }
        
        return newcount;
    }
    
    static inline float getNCC( uchar *targetPtr, uchar *templatePtr, float templateA, float templateC )
    {
        unsigned int A = computeSum( targetPtr );
        unsigned int B = computeSumSq( targetPtr );
        unsigned int D = computeDotProduct( targetPtr, templatePtr );
        
        float Af = A;
        float Bf = B;
        float Cf = 1.f / sqrtf( 64 * Bf - Af * Af );
        float Df = D;
        float score = ( 64 * Df - Af * templateA ) * Cf * templateC;
        return score;
    }
    
    static inline float getNCC( uchar *targetPtr, int targetRowStep, uchar *templatePtr, float templateA, float templateC )
    {
        unsigned int A = computeSum( targetPtr, targetRowStep );
        unsigned int B = computeSumSq( targetPtr, targetRowStep );
        unsigned int D = computeDotProduct( targetPtr, targetRowStep, templatePtr );
        
        float Af = A;
        float Bf = B;
        float Cf = 1.f / sqrtf( 64 * Bf - Af * Af );
        float Df = D;
        float score = ( 64 * Df - Af * templateA ) * Cf * templateC;
        return score;
    }
    
    void searchNCCTemplate( void *context, size_t i )
    {
        PatchSearchNCC *searcher = (PatchSearchNCC*)context;
        Patch *patch = *(searcher->begin+i);
        
        size_t index = patch->index;
        
        const int w = patch->target->image.size().width;
        
        uchar *templatePtr = searcher->templates + index*64;
        float *Aptr = searcher->templateA + index;
        float *Cptr = searcher->templateC + index;
        
        uchar *targetPtr;
        int targetRowStep;
        
        if ( searcher->subsample ) {
            targetPtr = searcher->targets + index*64;
            targetRowStep = 8;
        } else {
            targetPtr = NULL;
            targetRowStep = w;
        }
        
        cv::Mat templatePatch( cv::Size(8,8), CV_8UC1, templatePtr );
        cv::Mat targetPatch( cv::Size(8,8), CV_8UC1, targetPtr );
        
        cv::Point2i bestLoc(0,0);
        float nextBestScore = -INFINITY;
        float bestScore = 0;
        int num_inliers = 0;
        
        Eigen::Vector2f offset;
        offset << 3.5, 3.5;
        Eigen::Vector2f center = patch->center - offset;
        
        cv::Point2i ir_offset( 4, 4 );
        cv::Point2i ir_origin = cv::Point2i( (int) round( center[0] ), (int) round( center[1] ) ) - ir_offset;
        
        Eigen::Matrix<float,8,8> scores = Eigen::Matrix<float,8,8>::Zero();
        
        int lower = 0;
        int upper = 8;
        
        for ( int y = lower; y < upper; y++ ) {
            for ( int x = lower; x < upper; x++ ) {
                if ( searcher->subsample ) {
                    Eigen::Vector2f pt;
                    pt[0] = center[0] + x;
                    pt[1] = center[1] + y;
                    bool good = patch->sampler.samplePatch( patch->target->image, pt, targetPatch );
                    if ( !good ) continue;
                } else {
                    cv::Point2i my_origin = ir_origin + cv::Point2i(x,y);
                    if ( !in_image( patch->target->image.size(), my_origin ) ) continue;
                    if ( !in_image( patch->target->image.size(), my_origin + cv::Point2i(targetPatch.size().width,targetPatch.size().height) ) ) continue;
                    
                    targetPtr = patch->target->image.ptr( my_origin.y ) + my_origin.x;
                }
                
                float score = getNCC( targetPtr, targetRowStep, templatePtr, (*Aptr), (*Cptr) );
                
                if ( std::isnan( score ) ) score = 0.f;
                scores(y,x) = score;
                
                if ( score >= searcher->lowThreshold )
                {
                    num_inliers++;
                }
                if ( score > bestScore )
                {
                    bestLoc = cv::Point2i(x,y);
                    nextBestScore = bestScore;
                    bestScore = score;
                }
            }
        }
        
        if ( num_inliers == 0 ) {
            patch->shouldTrack = false;
            return;
        }

        patch->bestLevel = searcher->level;
        
        if ( searcher->subsample ) {
            patch->targetPos[0] = center[0] + bestLoc.x;
            patch->targetPos[1] = center[1] + bestLoc.y;
        } else {
            patch->targetPos[0] = (ir_origin.x + ir_offset.x) + bestLoc.x;
            patch->targetPos[1] = (ir_origin.y + ir_offset.y) + bestLoc.y;
        }
        
        
        patch->targetScore = bestScore;
    }
    
    int PatchSearchNCC::doSearch( int count )
    {
#ifdef USE_DISPATCH
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
        dispatch_apply_f( count, queue, this, searchNCCTemplate );
#else
        for ( int i = 0; i < count; i++ ) {
            searchTimer.start();
            searchNCCTemplate( this, i );
            searchTimer.stop();
        }
#endif
        
        int newcount = 0;
        for ( int i = 0; i < count; i++ )
        {
            Patch *patch = *(begin+i);
            if ( patch->bestLevel == level ) newcount++;
        }
        
        return newcount;
    }
}
