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

#include <opencv2/imgproc.hpp>

#include <Eigen/Core>

namespace vrlt
{
    
    ImageCache::ImageCache()
    : matchThreshold( 30 * 30 * 80 * 45 ), maxCacheSize( 1 ), next_to_try( 0 ), ssd_calc( cv::Size(80,45) )
    {
        
    }
    
    void ImageCache::makeSmallByteImage( Camera *camera )
    {
        cv::Mat img1;
        cv::Mat img2;
        cv::Mat img3;
        cv::pyrDown( camera->image, img1 );
        cv::pyrDown( img1, img2 );
        cv::pyrDown( img2, img3 );
        cv::pyrDown( img3, camera->small_byte_image );
    }
    
    void ImageCache::prepareSmallImage( Camera *camera )
    {
        cv::Mat img5;
        cv::pyrDown( camera->small_byte_image, img5 );
        camera->small_byte_image = img5;

        camera->small_byte_image.convertTo( camera->small_image, CV_32FC1 );

        cv::GaussianBlur( camera->small_image, camera->small_image, cv::Size(7,7), 2.5 );
        cv::subtract( camera->small_image, cv::mean(camera->small_image), camera->small_image );
        cv::divide( camera->small_image, cv::norm(camera->small_image), camera->small_image );
    }
    
    bool ImageCache::test( Camera *camera )
    {
        Sophus::SE3d inv_pose = camera->node->globalPose().inverse();
        
        for ( int i = 0; i < cache.size(); i++ )
        {
            Sophus::SE3d rel_pose = cache[i]->node->globalPose() * inv_pose;
            double dist = rel_pose.translation().norm();
            if ( dist < 0.1 ) {
                double angle = rel_pose.so3().log().norm() * 180. / M_PI;
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