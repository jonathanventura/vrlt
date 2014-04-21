/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: features.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <FeatureExtraction/features.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/nonfree.hpp>

#include <iostream>

namespace vrlt {
	
    static cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt)
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

    static uchar getGraySubpix(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 1);
        
        int x = (int)pt.x;
        int y = (int)pt.y;
        
        int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
        int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
        int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
        int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);
        
        float a = pt.x - (float)x;
        float c = pt.y - (float)y;
        
        uchar b = (uchar)cvRound((img.at<uchar>(y0, x0) * (1.f - a) + img.at<uchar>(y0, x1) * a) * (1.f - c)
                                 + (img.at<uchar>(y1, x0) * (1.f - a) + img.at<uchar>(y1, x1) * a) * c);
        
        return b;
    }

    /*
    int detectShiTomasi( CVD::BasicImage<CVD::byte> &image, std::vector<Feature*> &features, int numcorners )
    {
        std::vector<ImageRef> corners_out;
        
        int count = 0;
        shitomasi_corner_detect( image, corners_out, numcorners );
        
        vector<ImageRef>::iterator it;
        for ( it = corners_out.begin(); it != corners_out.end(); it++ )
        {
            Feature *feature = new Feature;
            
            feature->location[0] = it->x;
            feature->location[1] = it->y;
            feature->scale = 4.;
            
            features.push_back( feature );
            
            count++;
        }
        
        return count;
    }

    int detectHarris( CVD::BasicImage<CVD::byte> &image, std::vector<Feature*> &features, int numcorners )
    {
        std::vector<ImageRef> corners_out;
        
        int old_count = features.size();
        harris_corner_detect( image, corners_out, numcorners );
        
        vector<ImageRef>::iterator it;
        for ( it = corners_out.begin(); it != corners_out.end(); it++ )
        {
            Feature *feature = new Feature;
            
            feature->location[0] = it->x;
            feature->location[1] = it->y;
            feature->scale = 1.6;
            
            features.push_back( feature );
        }
        
        return features.size() - old_count;
    }
    
    static int detectFASTcorners( CVD::BasicImage<CVD::byte> &image, std::vector<ImageRef> &corners_out, int thresh )
    {
        vector<ImageRef> corners;
        
        fast_corner_detect_10(image, corners, thresh);
        fast_nonmax( image, corners, thresh, corners_out );
        
        return (int)corners_out.size();
    }
    
    int detectFAST( CVD::BasicImage<CVD::byte> &image, std::vector<Feature*> &features, int thresh )
    {
        std::vector<ImageRef> corners_out;
        
        int count = detectFASTcorners( image, corners_out, thresh );
        
        vector<ImageRef>::iterator it;
        for ( it = corners_out.begin(); it != corners_out.end(); it++ )
        {
            Feature *feature = new Feature;
            
            feature->location[0] = it->x;
            feature->location[1] = it->y;
            feature->scale = 4.;
            
            features.push_back( feature );
        }
        
        return count;
    }
    
    int detectFAST( ImagePyramid &pyramid, std::vector<Feature*> &features )
    {
        int total_count = 0;
        
        for ( int i = 0; i < 4; i++ )
        {
            std::vector<ImageRef> corners_out;
            
            int count = detectFASTcorners( pyramid.levels[i].image, corners_out, 10 );
            total_count += count;
            
            vector<ImageRef>::iterator it;
            for ( it = corners_out.begin(); it != corners_out.end(); it++ )
            {
                Feature *feature = new Feature;
                
                double scale = pow( 2., i );
                feature->location[0] = ( it->x + .5 ) * scale - .5;
                feature->location[1] = ( it->y + .5 ) * scale - .5;
                feature->scale = 4. * scale;
                
                features.push_back( feature );
            }
        }
        
        return total_count;
    }
    
    int extractPatchDescriptors( BasicImage< Rgb<byte> > &color_image, vector<Feature*> &features_in, vector<Feature*> &features_out )
    {
        Image<byte> image( color_image.size() );
        image_interpolate<Interpolate::Bilinear,Rgb<byte> > color_interpolator( color_image );

        convert_image( color_image, image );
        
        ImagePyramid pyramid( image.size() );
        pyramid.copy_from( image );
        
        int nadded = 0;
        for ( int i = 0; i < features_in.size(); i++ ) 
        {
            Feature *feature_in = features_in[i];
            Feature *feature_out = new Feature;

            feature_out->track = feature_in->track;
            feature_out->camera = feature_in->camera;
            feature_out->location = feature_in->location;
            feature_out->scale = feature_in->scale;
            feature_out->orientation = 0;
            Rgb<float> color = color_interpolator[ feature_out->location ];
            feature_out->color[0] = color.red;
            feature_out->color[1] = color.green;
            feature_out->color[2] = color.blue;

            int level = feature_in->scale;
            if ( level < 0 ) level = 0;
            if ( level > 3 ) level = 3;

            double scale = pow( 2., level );
            Eigen::Vector2d center = feature_in->location;
            center[0] = ( center[0] + .5 ) / scale - .5;
            center[1] = ( center[1] + .5 ) / scale - .5;
            
            Eigen::Vector2d origin = center - makeVector( 7.5, 7.5 );
            
            feature_out->descriptor = new unsigned char[128];
            bzero( feature_out->descriptor, 128 );
            
            image_interpolate<Interpolate::Bilinear,byte> interpolator( pyramid.levels[level].image );

            int n = 0;
            bool good = true;
            for ( int y = -5; y <= 5; y++ ) {
                for ( int x = -5; x <= 5; x++,n++ ) {
                    Eigen::Vector2d loc = origin + makeVector(x,y);
                    if ( !interpolator.in_image(loc) ) {
                        good = false;
                        break;
                    }
                    feature_out->descriptor[n] = interpolator[loc];
                }
                if ( !good ) break;
            }
            
            if ( !good ) {
                delete feature_out;
                continue;
            }

            nadded++;
            features_out.push_back( feature_out );
        }
        
        return nadded;
    }
    */

    void normalizeFloats( float *floatdata )
    {
        float norm = 0;
        for ( int i = 0; i < 128; i++ ) norm += floatdata[i] * floatdata[i];
        norm = sqrt(norm);
        for ( int i = 0; i < 128; i++ ) floatdata[i] /= norm;
    }
    
    void limitFloats( float *floatdata )
    {
        for ( int i  = 0; i < 128; i++ ) {
            if ( floatdata[i] > 0.2 ) floatdata[i] = 0.2;
        }
    }
    
    /*
    int extractSIFTdescriptors( cv::Mat &image, vector<Feature*> &features_in, vector<Feature*> &features_out )
    {
        int err;
        int count = 0;
        
        cv::SIFT sift;
        
        cv::Mat gray_image;
        if ( image.type() == CV_8UC3 )
        {
            cv::cvtColor( image, gray_image, cv::COLOR_RGB2GRAY );
        }
        else if ( image.type() == CV_8UC1 )
        {
            gray_image = image;
        }
        
        std::vector<cv::KeyPoint> keypoints( features_in.size() );
        for ( size_t i = 0; i < features_in.size(); i++ )
        {
            keypoints[i].
        }
        sift( gray_image, cv::noArray(), keypoints, descriptors );
        
        
        Image<vl_sift_pix> floatim( image.size() );
        
        // convert image to grayscale float
        image_interpolate<Interpolate::Bilinear,byte > interpolator( image );
        convert_image( image, floatim );
        
        // allocate sift structure
        VlSiftFilt* f = vl_sift_new( image.size().x, image.size().y, -1, 3, 0 );
        
        // create keypoints
        // this determines the octave for processing
        int nfeatures = (int)features_in.size();
        VlSiftKeypoint *keys = new VlSiftKeypoint[nfeatures];
        for ( int i = 0; i < nfeatures; i++ )
        {
            Feature *feature = features_in[i];
            vl_sift_keypoint_init( f, keys+i, feature->location[0], feature->location[1], feature->scale );
        }
        
        // get descriptors
        float floatdata[128];
        err = vl_sift_process_first_octave( f, floatim.data() );
        do {	
            for ( int i = 0; i < nfeatures; i++ ) {
                VlSiftKeypoint *key = keys+i;
                
                if ( key->o != f->o_cur ) continue;
                
                double angles[4];
                int nangles = vl_sift_calc_keypoint_orientations( f, angles, key );
                
                Feature *feature_in = features_in[i];
                
                for ( int j = 0; j < nangles; j++ ) {
                    count++;
                    
                    Feature *feature_out = new Feature;
                    feature_out->track = feature_in->track;
                    feature_out->camera = feature_in->camera;
                    feature_out->location = feature_in->location;
                    feature_out->scale = key->sigma;
                    feature_out->orientation = angles[j];
                    float color = interpolator[ feature_out->location ];
                    feature_out->color[0] = color;
                    feature_out->color[1] = color;
                    feature_out->color[2] = color;
                    
                    vl_sift_calc_keypoint_descriptor( f, floatdata, key, angles[j] );
                    
                    // they seem to be already normalized
                    //normalizeFloats( floatdata );
                    
                    feature_out->descriptor = new unsigned char[128];
                    for ( int k = 0; k < 128; k++ ) {
                        float val = floatdata[k] * 512.f;
                        //float val = floatdata[k] * 512.f * 2.f;
                        if ( val > 255.f ) val = 255.f;
                        feature_out->descriptor[k] = (unsigned char) val;
                    }
                    
                    features_out.push_back( feature_out );
                }
            }
            err = vl_sift_process_next_octave( f );
        } while ( err != VL_ERR_EOF );
        
        delete [] keys;
        vl_sift_delete( f );
        
        return count;
    }
    
    int extractSIFTdescriptors( BasicImage< Rgb<byte> > &color_image, vector<Feature*> &features_in, vector<Feature*> &features_out )
    {
        int err;
        int count = 0;
        
        Image<byte> image( color_image.size() );
        Image<vl_sift_pix> floatim( image.size() );
        
        // convert image to grayscale float
        image_interpolate<Interpolate::Bilinear,Rgb<byte> > interpolator( color_image );
        convert_image( color_image, image );
        convert_image( image, floatim );
        
        // allocate sift structure
        VlSiftFilt* f = vl_sift_new( image.size().x, image.size().y, -1, 3, 0 );
        
        // create keypoints
        // this determines the octave for processing
        int nfeatures = (int)features_in.size();
        VlSiftKeypoint *keys = new VlSiftKeypoint[nfeatures];
        for ( int i = 0; i < nfeatures; i++ )
        {
            Feature *feature = features_in[i];
            vl_sift_keypoint_init( f, keys+i, feature->location[0], feature->location[1], feature->scale );
        }

        // get descriptors
        float floatdata[128];
        err = vl_sift_process_first_octave( f, floatim.data() );
        do {	
            for ( int i = 0; i < nfeatures; i++ ) {
                VlSiftKeypoint *key = keys+i;
                
                if ( key->o != f->o_cur ) continue;
                
                double angles[4];
                int nangles = vl_sift_calc_keypoint_orientations( f, angles, key );
                
                Feature *feature_in = features_in[i];
                
                for ( int j = 0; j < nangles; j++ ) {
                    count++;
                    
                    Feature *feature_out = new Feature;
                    feature_out->track = feature_in->track;
                    feature_out->camera = feature_in->camera;
                    feature_out->location = feature_in->location;
                    feature_out->scale = key->sigma;
                    feature_out->orientation = angles[j];
                    Rgb<float> color = interpolator[ feature_out->location ];
                    feature_out->color[0] = color.red;
                    feature_out->color[1] = color.green;
                    feature_out->color[2] = color.blue;
                    
                    vl_sift_calc_keypoint_descriptor( f, floatdata, key, angles[j] );
                    
                    // they seem to be already normalized
                    //normalizeFloats( floatdata );
                    
                    feature_out->descriptor = new unsigned char[128];
                    for ( int k = 0; k < 128; k++ ) {
                        float val = floatdata[k] * 512.f;
                        //float val = floatdata[k] * 512.f * 2.f;
                        if ( val > 255.f ) val = 255.f;
                        feature_out->descriptor[k] = (unsigned char) val;
                    }
                    
                    features_out.push_back( feature_out );
                }
            }
            err = vl_sift_process_next_octave( f );
        } while ( err != VL_ERR_EOF );
        
        delete [] keys;
        vl_sift_delete( f );
        
        return count;
    }
    */
    
    int extractSIFT( cv::Mat &image, std::vector<Feature*> &features, int o_min, double contrast_thresh )
    {
        cv::Mat gray_image;
        if ( image.channels() == 3 )
        {
            cv::cvtColor( image, gray_image, cv::COLOR_RGB2GRAY );
        }
        else
        {
            gray_image = image;
        }
        
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        
        cv::SIFT sift( 0, 3, contrast_thresh );
        sift( gray_image, cv::noArray(), keypoints, descriptors );
        
        float *floatdata = (float*)descriptors.ptr();
        
        features.clear();
        features.reserve( keypoints.size() );
        for ( size_t i = 0; i < keypoints.size(); i++,floatdata+=128 )
        {
            // according to this page:
            // https://groups.google.com/forum/#!topic/javacv/E9xngMMwVW4
            // SIFT octave is found by (keypoint.octave & 255)
            if ( (keypoints[i].octave & 255) < o_min ) continue;
            
            Feature *feature = new Feature;
            feature->location[0] = keypoints[i].pt.x;
            feature->location[1] = keypoints[i].pt.y;
            feature->scale = keypoints[i].size;
            feature->orientation = keypoints[i].angle;
            
            if ( image.channels() == 3 )
            {
                cv::Vec3b color = getColorSubpix( image, keypoints[i].pt );
                feature->color[0] = color[0];
                feature->color[1] = color[1];
                feature->color[2] = color[2];
            }
            else
            {
                uchar gray = getGraySubpix( image, keypoints[i].pt );
                feature->color[0] = gray;
                feature->color[1] = gray;
                feature->color[2] = gray;
            }
            
            normalizeFloats( floatdata );

            feature->descriptor = new unsigned char[128];
            for ( int k = 0; k < 128; k++ ) {
                float val = floatdata[k] * 512.f;
                if ( val > 255.f ) val = 255.f;
                feature->descriptor[k] = (unsigned char) val;
            }
            features.push_back( feature );
        }
        
        return features.size();
    }
}

