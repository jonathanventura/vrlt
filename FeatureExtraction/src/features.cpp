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

#include <cvd/fast_corner.h>
#include <cvd/harris_corner.h>

#include <cvd/image_convert.h>
#include <cvd/image_interpolate.h>

#ifdef USE_VL
extern "C" { 
#include <vl/sift.h>
}
#endif

namespace vrlt {
	
	using namespace TooN;
	using namespace CVD;
	using namespace std;
        
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
            Vector<2> center = feature_in->location;
            center[0] = ( center[0] + .5 ) / scale - .5;
            center[1] = ( center[1] + .5 ) / scale - .5;
            
            Vector<2> origin = center - makeVector( 7.5, 7.5 );
            
            feature_out->descriptor = new unsigned char[128];
            bzero( feature_out->descriptor, 128 );
            
            image_interpolate<Interpolate::Bilinear,byte> interpolator( pyramid.levels[level].image );

            int n = 0;
            bool good = true;
            for ( int y = -5; y <= 5; y++ ) {
                for ( int x = -5; x <= 5; x++,n++ ) {
                    Vector<2> loc = origin + makeVector(x,y);
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
    

#ifdef USE_VL
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
    
    int extractSIFTdescriptors( BasicImage<byte> &image, vector<Feature*> &features_in, vector<Feature*> &features_out )
    {
        int err;
        int count = 0;
        
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
    
    int extractSIFT( BasicImage<byte> &image, vector<Feature*> &features, int o_min, bool upright, float peak_thresh )
    {
        Image<vl_sift_pix> floatim( image.size() );
        for ( int i = 0; i < image.size().x * image.size().y; i++ ) floatim.data()[i] = (float) image.data()[i];
        
        VlSiftFilt* f = vl_sift_new( image.size().x, image.size().y, -1, 3, o_min );
        vl_sift_set_peak_thresh( f, peak_thresh );
        
        int err;
        
        int count = 0;
        
        err = vl_sift_process_first_octave( f, floatim.data() );
        do {	
            vl_sift_detect( f );
            
            const VlSiftKeypoint* keys = vl_sift_get_keypoints( f );
            const VlSiftKeypoint* key = keys;
            float floatdata[128];
            
            int nkeypoints = vl_sift_get_nkeypoints( f );
            for ( int i = 0; i < nkeypoints; i++,key++ ) {
                double angles[4];
                
                int nangles = 1;
                angles[0] = 0;
                
                if ( !upright ) nangles = vl_sift_calc_keypoint_orientations( f, angles, key );
                
                for ( int j = 0; j < nangles; j++ ) {
                    count++;
                    
                    Feature *feature = new Feature;
                    feature->location[0] = key->x;
                    feature->location[1] = key->y;
                    feature->scale = key->sigma;
                    feature->orientation = angles[j];
                    
                    feature->color[0] = 0;
                    feature->color[1] = 0;
                    feature->color[2] = 0;
                    
                    vl_sift_calc_keypoint_descriptor( f, floatdata, key, angles[j] );
                    
                    normalizeFloats( floatdata );
                    /*
                    limitFloats( floatdata );
                    normalizeFloats( floatdata );
                    */
                    feature->descriptor = new unsigned char[128];
                    for ( int k = 0; k < 128; k++ ) {
                        float val = floatdata[k] * 512.f;
                        //float val = floatdata[k] * 512.f * 2.f;
                        if ( val > 255.f ) val = 255.f;
                        feature->descriptor[k] = (unsigned char) val;
                    }
                    
                    features.push_back( feature );
                }
            }
            err = vl_sift_process_next_octave( f );
        } while ( err != VL_ERR_EOF );
        
        vl_sift_delete( f );

        return count;
    }
    
    int extractSIFT( BasicImage< Rgb<byte> > &color_image, vector<Feature*> &features, int o_min, bool upright, float peak_thresh )
    {
        image_interpolate<Interpolate::Bilinear,Rgb<byte> > interpolator( color_image );
        
        Image<byte> image( color_image.size() );
        convert_image( color_image, image );
        
        Image<vl_sift_pix> floatim( image.size() );
        //convert_image( image, floatim );
        
        for ( int i = 0; i < image.size().x * image.size().y; i++ ) floatim.data()[i] = (float) image.data()[i];
        
        VlSiftFilt* f = vl_sift_new( image.size().x, image.size().y, -1, 3, o_min );
        vl_sift_set_peak_thresh( f, peak_thresh );

        int err;
        
        int count = 0;
        
        err = vl_sift_process_first_octave( f, floatim.data() );
        do {	
            vl_sift_detect( f );
            
            const VlSiftKeypoint* keys = vl_sift_get_keypoints( f );
            const VlSiftKeypoint* key = keys;
            float floatdata[128];
            
            int nkeypoints = vl_sift_get_nkeypoints( f );
            for ( int i = 0; i < nkeypoints; i++,key++ ) {
                double angles[4];
                
                int nangles = 1;
                angles[0] = 0;
                
                if ( !upright ) nangles = vl_sift_calc_keypoint_orientations( f, angles, key );
                
                for ( int j = 0; j < nangles; j++ ) {
                    count++;
                    
                    Feature *feature = new Feature;
                    feature->location[0] = key->x;
                    feature->location[1] = key->y;
                    feature->scale = key->sigma;
                    feature->orientation = angles[j];
                    
                    Rgb<float> color;
                    if ( interpolator.in_image( feature->location ) ) color = interpolator[ feature->location ];
                    feature->color[0] = color.red;
                    feature->color[1] = color.green;
                    feature->color[2] = color.blue;

                    vl_sift_calc_keypoint_descriptor( f, floatdata, key, angles[j] );
                    
                    normalizeFloats( floatdata );
                    /*
                    limitFloats( floatdata );
                    normalizeFloats( floatdata );
                    */
                    feature->descriptor = new unsigned char[128];
                    for ( int k = 0; k < 128; k++ ) {
                        float val = floatdata[k] * 512.f;
                        if ( val > 255.f ) val = 255.f;
                        feature->descriptor[k] = (unsigned char) val;
                    }
                    features.push_back( feature );
                }
            }
            err = vl_sift_process_next_octave( f );
        } while ( err != VL_ERR_EOF );
        
        vl_sift_delete( f );
        
        return count;
    }
#endif
}

