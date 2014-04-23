
#ifndef LINES_H
#define LINES_H

#include <MultiView/multiview.h>

namespace vrlt
{
    struct Line
    {
        Camera *source;
        
        Eigen::Vector3d pt1;
        Eigen::Vector3d pt2;
        
        Eigen::Vector3d coeffs;
        double score;
        
        Line( Camera *_source, const Eigen::Vector3d &_pt1, const Eigen::Vector3d &_pt2 )
        : source( _source ), pt1( _pt1 ), pt2( _pt2 )
        {
            score = ( pt1 - pt2 ).norm();
            coeffs = pt2.cross( pt1 );
            coeffs.normalize();
        }
    };

    struct SortLines
    {
        bool operator()( const Line &a, const Line &b ) { return a.score > b.score; }
    };
    
    void drawLines( Node *node, std::vector<Line> &lines );
    void extractLines( Node *node, std::vector<Line> &vert_lines, std::vector<Line> &horz_lines, double min_length );
    Sophus::SO3d rectify( Node *node, double min_length );
    
}

#endif
