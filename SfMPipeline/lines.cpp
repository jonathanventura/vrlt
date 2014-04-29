
#include <Estimator/estimator.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

#include "lines.h"

namespace vrlt
{
    void drawLines( Node *node, std::vector<Line> &lines )
    {
        if ( node->camera ) {
            Camera *camera = node->camera;
            
            cv::Mat image_out = cv::imread( camera->path, cv::IMREAD_GRAYSCALE );
            
            for ( int i = 0; i < lines.size(); i++ ) 
            {
                if ( lines[i].source != camera ) continue;
                Eigen::Vector2d mypt1 = camera->calibration->project( project( node->globalRotation() * lines[i].pt1 ) );
                Eigen::Vector2d mypt2 = camera->calibration->project( project( node->globalRotation() * lines[i].pt2 ) );
                cv::line( image_out, cv::Point2i(mypt1[0], mypt1[1]), cv::Point2i(mypt2[0], mypt2[1]), cv::Scalar(255) );
            }
            
            char name[256];
            sprintf( name, "Output/lines%s.png", camera->name.c_str() );
            cv::imwrite( name, image_out );
        }
        
        ElementList::iterator it;
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node *)it->second;
            drawLines( child, lines );
        }
    }
    
    void extractLines( Node *node, std::vector<Line> &vert_lines, std::vector<Line> &horz_lines, double min_length )
    {
        if ( node->camera != NULL )
        {
            cv::Mat image = cv::imread( node->camera->path, cv::IMREAD_GRAYSCALE );
            
            cv::Ptr<cv::LineSegmentDetector> detector = cv::createLineSegmentDetector();
            
            std::vector<cv::Vec4i> lines;
            detector->detect( image, lines );
            
            for ( size_t i = 0; i < lines.size(); i++ )
            {
                Eigen::Vector2d pt1;
                pt1 << lines[i][0], lines[i][1];
                Eigen::Vector2d pt2;
                pt2 << lines[i][2], lines[i][3];
                
                double dist = ( pt2 - pt1 ).norm();
                if ( dist < min_length ) continue;
                
                double angle = fabs( atan2( pt2[1] - pt1[1], pt2[0] - pt1[0] ) );
                bool vertical = angle > M_PI / 4 && angle < 3 * M_PI / 4;
                
                Eigen::Vector3d worldpt1 = node->globalRotation().inverse() * node->camera->calibration->unproject( pt1 );
                Eigen::Vector3d worldpt2 = node->globalRotation().inverse() * node->camera->calibration->unproject( pt2 );
                if ( vertical ) vert_lines.push_back( Line( node->camera, worldpt1, worldpt2 ) );
                else horz_lines.push_back( Line( node->camera, worldpt1, worldpt2 ) );
            }
        }
        
        ElementList::iterator it;
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node *)it->second;
            extractLines( child, vert_lines, horz_lines, min_length );
        }
    }
    
    Sophus::SO3d rectify( Node *node, double min_length, bool rotated )
    {
        // extract lines
        std::vector<Line> vert_lines, horz_lines;
        Eigen::Vector3d up;
        if ( rotated )
        {
            extractLines( node, horz_lines, vert_lines, min_length );
            up << 0, 0, 1;
        }
        else
        {
            extractLines( node, vert_lines, horz_lines, min_length );
            up << 0, 1, 0;
        }
        std::sort( vert_lines.begin(), vert_lines.end(), SortLines() );
        
        PROSAC prosac;
        prosac.min_num_inliers = (int)vert_lines.size();
        prosac.num_trials = 20000;
        prosac.inlier_threshold = 2.;
        std::vector<bool> inliers;
        
        // make vertical line list
        // just put the line coeffs in the first part
        PointPairList line_list;
        for ( int i = 0; i < vert_lines.size(); i++ )
        {
            line_list.push_back( std::make_pair( vert_lines[i].coeffs, Eigen::Vector3d::Zero() ) );
        }
        
        // estimate vertical vanishing point
        VanishingPoint vvp( up );
        int ninliers = prosac.compute( line_list.begin(), line_list.end(), vvp, inliers );
        
        std::cout << node->name << " vertical estimation: " << ninliers << " / " << vert_lines.size() << "\n";
        std::cout << "vanishing point: " << vvp.vp.transpose() << "\n";
        std::cout << "corrected: " << (vvp.R * vvp.vp).transpose() << "\n";
        
        // get rotation
        Sophus::SO3d Rvert = vvp.R;
        
        std::vector<Line> inlier_lines;
        for ( int i = 0; i < vert_lines.size(); i++ )
        {
            if ( inliers[i] ) {
                Line myline = vert_lines[i];
                myline.coeffs = vvp.R.inverse() * myline.coeffs;
                myline.pt1 = vvp.R * myline.pt1;
                myline.pt2 = vvp.R * myline.pt2;
                inlier_lines.push_back( myline );
            }
        }
        
        drawLines( node, inlier_lines );
        
        if ( rotated )
        {
            Eigen::Matrix3d Z_to_negY;
            Z_to_negY <<
            1,0,0,
            0,0,-1,
            0,1,0;
            Rvert = Sophus::SO3d(Z_to_negY)*Rvert;
        }

        return Rvert;
    }
}