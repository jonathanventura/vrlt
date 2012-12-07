/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: imagecache.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <ImageCache/imagecache.h>
#include <PatchTracker/ssd.h>

#include <cvd/vision.h>
#include <cvd/image_convert.h>
#include <cvd/convolution.h>
#include <cvd/image_io.h>

#include <TooN/TooN.h>

namespace vrlt
{
    using namespace std;
    using namespace TooN;
    using namespace CVD;
    
    ImageCache::ImageCache()
    : matchThreshold( 30 * 30 * 80 * 45 ), maxCacheSize( 1 ), ssd_calc( ImageRef(80,45) ), next_to_try( 0 )
    {
        
    }
    
    void ImageCache::makeSmallByteImage( Camera *camera )
    {
        camera->small_byte_image = halfSample( halfSample( halfSample( halfSample( camera->image ) ) ) );
    }
    
    void ImageCache::prepareSmallImage( Camera *camera )
    {
        camera->small_byte_image = halfSample( camera->small_byte_image );

        camera->small_image.resize( camera->small_byte_image.size() );
        convert_image( camera->small_byte_image, camera->small_image );

        convolveGaussian( camera->small_image, 2.5 );
        removeMean( camera->small_image );
        normalize( camera->small_image );
    }
    
    bool ImageCache::test( Camera *camera )
    {
        SE3<> inv_pose = camera->node->globalPose().inverse();
        
        for ( int i = 0; i < cache.size(); i++ )
        {
            SE3<> rel_pose = cache[i]->node->globalPose() * inv_pose;
            double dist = norm( rel_pose.get_translation() );
            if ( dist < 0.1 ) {
                double angle = norm( rel_pose.get_rotation().ln() ) * 180. / M_PI;
                if ( angle < 45. ) return false;
            }
        }
        return true;
    }
    
    void ImageCache::add( Camera *camera )
    {
        if ( cache.size() >= maxCacheSize )
        {
            cache.erase( cache.begin() );
        }
        cache.push_back( camera );
        next_to_try = cache.size() - 1;
    }
    
    void ImageCache::addCopy( Camera *camera )
    {
        Camera *mycamera = new Camera;
        mycamera->name = camera->name;
        mycamera->calibration = camera->calibration;
        mycamera->image = camera->image;
        Node *mynode = new Node;
        mynode->name = camera->node->name;
        mycamera->node = mynode;
        mynode->camera = mycamera;
        mynode->pose = camera->node->pose;
        
        makeSmallByteImage( mycamera );
        prepareSmallImage( mycamera );
        this->add( mycamera );
    }
    
    void ImageCache::getNext( Camera *query )
    {
        if ( cache.empty() ) return;
        Camera *camera = cache[next_to_try];
        query->node->pose = camera->node->pose;
    }
    
    void ImageCache::moveNext()
    {
        next_to_try--;
        if ( next_to_try < 0 ) next_to_try = cache.size() - 1;
    }
    
    bool ImageCache::search( Camera *query )
    {
        makeSmallByteImage( query );
        prepareSmallImage( query );
        
        float bestCorr = -1.f;
        bestCamera = NULL;
        
        for ( int i = 0; i < cache.size(); i++ )
        {
            Camera *camera = cache[i];
            
            float corr = getCorr( camera->small_image, query->small_image );
            if ( corr > bestCorr )
            {
                bestCorr = corr;
                bestCamera = camera;
            }
        }
        
        if ( bestCamera == NULL ) return false;
        
        query->node->pose = bestCamera->node->pose;
        return true;
    }
}