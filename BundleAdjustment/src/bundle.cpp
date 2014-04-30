/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: bundle.cpp
 * Author: Jonathan Ventura
 * Last Modified: 20.04.2014
 */

#include <BundleAdjustment/bundle.h>
#include <BundleAdjustment/outliers.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>

namespace vrlt
{
    struct ReprojectionError
    {
        ReprojectionError( const Sophus::SE3d &_prefix, double _focal, double _x, double _y )
        : focal(_focal), x(_x), y(_y)
        {
            Eigen::Map<Eigen::Vector3d> ptrvec(prefix);
            ptrvec = _prefix.translation();
            ceres::RotationMatrixToAngleAxis( _prefix.so3().matrix().data(), prefix+3 );
        }
        
        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        T* residuals) const
        {
            // camera pose
            // camera[3,4,5] are the angle-axis rotation.
            T p[3];
            ceres::AngleAxisRotatePoint(camera+3, point, p);
            // camera[0,1,2] are the translation.
            p[0] += camera[0]; p[1] += camera[1]; p[2] += camera[2];
            
            // prefix pose (e.g. internal camera in panoramic head)
            T myprefix[6];
            myprefix[0] = T(prefix[0]);
            myprefix[1] = T(prefix[1]);
            myprefix[2] = T(prefix[2]);
            myprefix[3] = T(prefix[3]);
            myprefix[4] = T(prefix[4]);
            myprefix[5] = T(prefix[5]);
            T pp[3];
            ceres::AngleAxisRotatePoint(myprefix+3, p, pp);
            pp[0] += myprefix[0]; pp[1] += myprefix[1]; pp[2] += myprefix[2];
            
            // projection
            T xp = pp[0] / pp[2];
            T yp = pp[1] / pp[2];
            
            // intrinsics
            T fxp = T(focal) * xp;
            T fyp = T(focal) * yp;
            
            // residuals
            residuals[0] = fxp - T(x);
            residuals[1] = fyp - T(y);
            
            return true;
        }
        
        double prefix[6];
        double focal, x, y;
    };

    struct UprightReprojectionError
    {
        UprightReprojectionError( const Sophus::SE3d &_prefix, double _focal, double _x, double _y )
        : focal(_focal), x(_x), y(_y)
        {
            Eigen::Map<Eigen::Vector3d> ptrvec(prefix);
            ptrvec = _prefix.translation();
            ceres::RotationMatrixToAngleAxis( _prefix.so3().matrix().data(), prefix+3 );
        }
        
        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        T* residuals) const
        {
            // camera pose
            // [0;camera[3];0] is the angle-axis rotation.
            T p[3];
            T myrot[3];
            myrot[0] = T(0);
            myrot[1] = camera[3];
            myrot[2] = T(0);
            ceres::AngleAxisRotatePoint(myrot, point, p);
            // camera[0,1,2] are the translation.
            p[0] += camera[0]; p[1] += camera[1]; p[2] += camera[2];
            
            // prefix pose (e.g. internal camera in panoramic head)
            T myprefix[6];
            myprefix[0] = T(prefix[0]);
            myprefix[1] = T(prefix[1]);
            myprefix[2] = T(prefix[2]);
            myprefix[3] = T(prefix[3]);
            myprefix[4] = T(prefix[4]);
            myprefix[5] = T(prefix[5]);
            T pp[3];
            ceres::AngleAxisRotatePoint(myprefix+3, p, pp);
            pp[0] += myprefix[0]; pp[1] += myprefix[1]; pp[2] += myprefix[2];
            
            // projection
            T xp = pp[0] / pp[2];
            T yp = pp[1] / pp[2];
            
            // intrinsics
            T fxp = T(focal) * xp;
            T fyp = T(focal) * yp;
            
            // residuals
            residuals[0] = fxp - T(x);
            residuals[1] = fyp - T(y);
            
            return true;
        }
        
        double prefix[6];
        double focal, x, y;
    };
    
    class GlogInitializer
    {
    public:
        GlogInitializer()
        {
            char argv0[] = "vrlt_bundleadjsutment";
            google::InitGoogleLogging(argv0);
        }
    };
    
    class BundleInternal
    {
        static GlogInitializer glog_initializer;
    public:
        BundleInternal( Node *_root, const ElementList &_fixedNodes, const ElementList &_fixedPoints, bool _verbose = false, bool _upright = false );
        ~BundleInternal();
        
        
        bool run();
        void run_str();
        void run_mot();

        Node *root;
        ElementList fixedNodes;
        ElementList fixedPoints;
        
        bool verbose;
        bool upright;
        int itmax;
        
        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        
        void init();
        
        void addPoints();
        void fillPoints();
        void updatePoints();
        
        void addNodes();
        void fillNodes();
        void updateNodes();
        
        void _addMeasurements( Node *node, int j );
        void addMeasurements();
        void fillMeasurements();
        
        bool _run();
        bool _run_str();
        bool _run_mot();
        
        inline char & getVisibility( int i, int j );
        inline Feature* & getFeature( int i, int j );
        Feature **features;
        
        std::vector<Node*> nodes;
        std::map<Node*,int> node2index;
        
        std::vector<Point*> points;
        std::map<Point*,int> point2index;
        
        std::map< int, std::map< int, Sophus::SE3d > > prefixes;
        
        int n;
        int ncon;
        int m;
        int mcon;
        int o;
        char *vmask;
        double *p;
        int cnp;
        int pnp;
        double *x;
        double *covx;
        int mnp;
    };
    
    GlogInitializer BundleInternal::glog_initializer;
    
    Bundle::Bundle( Node *_root, bool _verbose, bool _upright )
    {
        internal = new BundleInternal( _root, ElementList(), ElementList(), _verbose, _upright );
    }
    
    Bundle::Bundle( Node *_root, const ElementList &_fixedNodes, const ElementList &_fixedPoints, bool _verbose, bool _upright )
    {
        internal = new BundleInternal( _root, _fixedNodes, _fixedPoints, _verbose, _upright );
    }
    
    Bundle::~Bundle()
    {
        delete internal;
    }
    
    bool Bundle::run()
    {
        return internal->run();
    }
    
    BundleInternal::BundleInternal( Node *_root, const ElementList &_fixedNodes, const ElementList &_fixedPoints, bool _verbose, bool _upright )
    : root( _root ), fixedNodes( _fixedNodes ), fixedPoints( _fixedPoints ), verbose( _verbose ), upright( _upright ), itmax( 100 ), lossFunction( NULL )
    {
        // # camera params
        if ( upright )
        {
            cnp = 4;
        } else {
            cnp = 6;
        }
        
        // # point params
        pnp = 3;                
        
        addPoints();
        addNodes();
        
        vmask = new char[m*n];  // visibility mask
        features = new Feature*[m*n];
        p = new double[cnp*m+pnp*n];    // camera params followed by point params
        
        fillPoints();
        fillNodes();
        
        mnp = 2;                // # measurement params
        
        addMeasurements();
    
        x = new double[mnp*o];
        covx = NULL;
        
        fillMeasurements();
        
        if ( verbose ) {
            std::cout << n << " points (" << ncon << " fixed)\n";
            std::cout << m << " nodes (" << mcon << " fixed)\n";
            std::cout << o << " measurements\n";
        }
    }
    
    BundleInternal::~BundleInternal()
    {
        delete [] vmask;
        delete [] features;
        delete [] p;
        delete [] x;
    }
    
    inline char & BundleInternal::getVisibility( int i, int j )
    {
        return vmask[i*m+j];
    }
    
    inline Feature* & BundleInternal::getFeature( int i, int j )
    {
        return features[i*m+j];
    }
    
    void BundleInternal::addPoints()
    {
        n = 0;
        ncon = 0;
        
        ElementList::iterator it;
        for ( it = root->points.begin(); it!= root->points.end(); it++ )
        {
            Point *point = (Point*)it->second;
            if ( fixedPoints.count( point->name ) == 0 ) continue;
            points.push_back(point);
            point2index[point] = n;
            n++;
            ncon++;
        }
        for ( it = root->points.begin(); it!= root->points.end(); it++ )
        {
            Point *point = (Point*)it->second;
            if ( fixedPoints.count( point->name ) > 0 ) continue;
            points.push_back(point);
            point2index[point] = n;
            n++;
        }
    }
    
    void BundleInternal::fillPoints()
    {
        double *ptr = p + cnp*m;
        for ( int i = 0; i < n; i++,ptr+=pnp )
        {
            Point *point = points[i];
            Eigen::Map<Eigen::Vector3d> ptrvec(ptr);
            ptrvec = project( point->position );
        }
    }
    
    void BundleInternal::updatePoints()
    {
        double *ptr = p + cnp*m;
        for ( int i = 0; i < n; i++,ptr+=pnp )
        {
            Point *point = points[i];
            point->position << ptr[0], ptr[1], ptr[2], 1.;
        }
    }
    
    void BundleInternal::addNodes()
    {
        m = 0;
        mcon = 0;
        
        ElementList::iterator it;
        for ( it = root->children.begin(); it!= root->children.end(); it++ )
        {
            Node *node = (Node*)it->second;
            if ( fixedNodes.count( node->name ) > 0 || node->fixed )
            {
                nodes.push_back(node);
                node2index[node] = m;
                m++;
                mcon++;
            }
        }
        for ( it = root->children.begin(); it!= root->children.end(); it++ )
        {
            Node *node = (Node*)it->second;
            if ( fixedNodes.count( node->name ) == 0 && !node->fixed ) {
                nodes.push_back(node);
                node2index[node] = m;
                m++;
            }
        }
    }
    
    void BundleInternal::fillNodes()
    {
        double *ptr = p;
        for ( int j = 0; j < m; j++,ptr+=cnp )
        {
            Node *node = nodes[j];
            
            Eigen::Map<Eigen::Vector3d> ptrvec(ptr);
            ptrvec = node->pose.translation();
            
            if ( upright )
            {
                ptr[3] = node->pose.so3().log()[1];
            }
            else
            {
                ceres::RotationMatrixToAngleAxis( node->pose.so3().matrix().data(), ptr+3 );
            }
        }
    }
    
    void BundleInternal::updateNodes()
    {
        double *ptr = p;
        for ( int j = 0; j < m; j++,ptr+=cnp )
        {
            Node *node = nodes[j];
            
            node->pose.translation() = Eigen::Map<Eigen::Vector3d>(ptr);
            if ( upright )
            {
                Eigen::Vector3d r;
                r << 0,ptr[3],0;
                node->pose.so3() = Sophus::SO3d::exp(r);
            }
            else
            {
                Eigen::Matrix3d R;
                ceres::AngleAxisToRotationMatrix(ptr+3, R.data());
                node->pose.so3() = Sophus::SO3d(R);
            }
        }
    }
    
    void BundleInternal::_addMeasurements( Node *node, int j )
    {
        if ( lossFunction == NULL ) lossFunction = new ceres::HuberLoss( 4.0 );
        
        Camera *camera = node->camera;
        if ( camera != NULL )
        {
            Calibration *calibration = camera->calibration;
            
            ElementList::iterator featureit;
            for ( featureit = camera->features.begin(); featureit != camera->features.end(); featureit++ )
            {
                Feature *feature = (Feature*)featureit->second;
                Track *track = feature->track;
                if ( track == NULL ) continue;
                
                Point *point = track->point;
                if ( point == NULL ) continue;
                
                int i = point2index[point];
                
                prefixes[j][i] = camera->node->globalPose( nodes[j] );
                
                // here is where we lose the redundancy of measurements
                // only one-to-one mapping between points and nodes
                getVisibility( i, j ) = 1;
                getFeature( i, j ) = feature;
                
                if ( upright )
                {
                    UprightReprojectionError *reproj_error = new UprightReprojectionError(prefixes[j][i],
                                                                            camera->calibration->focal,
                                                                            feature->location[0]-camera->calibration->center[0],
                                                                            feature->location[1]-camera->calibration->center[1]);
                    
                    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<UprightReprojectionError, 2, 4, 3>(reproj_error);
                    problem.AddResidualBlock(cost_function, lossFunction, p+j*cnp, p+m*cnp+i*pnp );
                }
                else
                {
                    ReprojectionError *reproj_error = new ReprojectionError(prefixes[j][i],
                                                                   camera->calibration->focal,
                                                                   feature->location[0]-camera->calibration->center[0],
                                                                   feature->location[1]-camera->calibration->center[1]);
                    
                    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(reproj_error);
                    problem.AddResidualBlock(cost_function, lossFunction, p+j*cnp, p+m*cnp+i*pnp );
                }
                
                if ( j < mcon ) problem.SetParameterBlockConstant( p+j*cnp );
                if ( i < ncon ) problem.SetParameterBlockConstant( p+m*cnp+i*pnp );
            }
        }
        
        ElementList::iterator it;
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node*)it->second;
            _addMeasurements( child, j );
        }
    }
    
    void BundleInternal::addMeasurements()
    {
        bzero(vmask, m*n);
        bzero(features, m*n*sizeof(Feature*));
        
        for ( int j = 0; j < m; j++ )
        {
            _addMeasurements( nodes[j], j );
        }
        
        o = 0;
        for ( int i = 0; i < n; i++ ) {
            for ( int j = 0; j < m; j++ ) {
                if ( getVisibility(i,j) > 0 ) o++;
            }
        }
    }
    
    void BundleInternal::fillMeasurements()
    {
        double *xptr = x;
        
        for ( int i = 0; i < n; i++ )
        {
            for ( int j = 0; j < m; j++ )
            {
                Feature *feature = getFeature(i,j);
                if ( feature == NULL ) continue;
                Eigen::Map<Eigen::Vector2d> xptrvec( xptr );
                xptrvec = feature->location;
                xptr += 2;
            }
        }
    }
    
    bool BundleInternal::run()
    {
        bool should_update = _run();
        
        if ( should_update )
        {
            updatePoints();
            updateNodes();
        }
        
        return should_update;
    }
    
    bool BundleInternal::_run()
    {
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        //if ( verbose ) options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if ( verbose ) std::cout << summary.FullReport() << "\n";
        return ( summary.termination_type != ceres::FAILURE );
    }
    
    void applyScale( Node *root, double factor )
    {
        ElementList::iterator it;
        for ( it = root->children.begin(); it != root->children.end(); it++ )
        {
            Node *node = (Node*)it->second;
            Eigen::Vector3d translation = node->pose.translation();
            node->pose.translation() = translation * factor;
            applyScale( node, factor );
        }
        for ( it = root->points.begin(); it != root->points.end(); it++ )
        {
            Point *point = (Point*)it->second;
            Eigen::Vector4d position = point->position;
            position[3] /= factor;
            point->position = position;
        }
    }
    
    void fixScale( Node *root )
    {
        ElementList::iterator it;
        for ( it = root->children.begin(); it != root->children.end(); it++ )
        {
            Node *node = (Node*)it->second;
            if ( node->scaleFixed )
            {
                Eigen::Vector3d translation = node->pose.translation();
                double factor = 1.0 / translation.norm();
                std::cout << "factor: " << 1.0 / factor << "\n";
                applyScale( root, factor );
            }
        }
    }
    
    bool runBundle( Reconstruction *r )
    {
        if ( r->nodes.empty() ) return false;
        
        Node *rootnode = (Node*)r->nodes["root"];

        Bundle bundle( rootnode, true, r->upright );
        bool good = bundle.run();
        if ( !good ) return false;
        
        fixScale( rootnode );
        
        return true;
    }
}
