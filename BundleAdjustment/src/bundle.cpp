
#include <BundleAdjustment/bundle.h>
#include <BundleAdjustment/outliers.h>

#include <ceres/ceres.h>

#include <iostream>

namespace vrlt
{
    Bundle::Bundle( Node *_root, bool _upright, bool _verbose ) : verbose( _verbose ), itmax( 100 ), root( _root ), upright( false )
    {
        init();
    }
    
    Bundle::Bundle( Node *_root, const ElementList &_fixedNodes, const ElementList &_fixedPoints, bool _upright, bool _verbose )
    : verbose( _verbose ), itmax( 100 ), root( _root ),  upright( false ), fixedNodes( _fixedNodes ), fixedPoints( _fixedPoints )
    {
        init();
    }
        
    void Bundle::init()
    {
        // # camera params
        if ( upright ) {
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
    
    Bundle::~Bundle()
    {
        delete [] vmask;
        delete [] features;
        delete [] p;
        delete [] x;
    }
    
    inline char & Bundle::getVisibility( int i, int j )
    {
        return vmask[i*m+j];
    }
    
    inline Feature* & Bundle::getFeature( int i, int j )
    {
        return features[i*m+j];
    }
    
    void Bundle::addPoints()
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
    
    void Bundle::fillPoints()
    {
        double *ptr = p + cnp*m;
        for ( int i = 0; i < n; i++,ptr+=pnp )
        {
            Point *point = points[i];
            Eigen::Map<Eigen::Vector3d> ptrvec(ptr);
            ptrvec = project( point->position );
        }
    }
    
    void Bundle::updatePoints()
    {
        double *ptr = p + cnp*m;
        for ( int i = 0; i < n; i++,ptr+=pnp )
        {
            Point *point = points[i];
            point->position << ptr[0], ptr[1], ptr[2], 1.;
        }
    }
    
    void Bundle::addNodes()
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
    
    void Bundle::fillNodes()
    {
        double *ptr = p;
        for ( int j = 0; j < m; j++,ptr+=cnp )
        {
            Node *node = nodes[j];
            
            if ( upright ) {
                *ptr = node->pose.so3().log()[1];
                Eigen::Map<Eigen::Vector3d> ptrvec( ptr+1 );
                ptrvec = node->pose.translation();
            } else {
                Eigen::Map< Eigen::Matrix<double,6,1> > ptrvec(ptr);
                ptrvec = node->pose.log();
            }
        }
    }
    
    void Bundle::updateNodes()
    {
        double *ptr = p;
        for ( int j = 0; j < m; j++,ptr+=cnp )
        {
            Node *node = nodes[j];
            
            if ( upright ) {
                Eigen::Vector3d r;
                r << 0, *ptr, 0;
                node->pose.so3() = Sophus::SO3d::exp( r );
                node->pose.translation() = Eigen::Map<Eigen::Vector3d>(ptr+1);
            } else {
                node->pose = Sophus::SE3d::exp( Eigen::Map< Eigen::Matrix<double,6,1> >(ptr) );
            }
        }
    }
    
    void Bundle::_addMeasurements( Node *node, int j )
    {
        Camera *camera = node->camera;
        if ( camera != NULL )
        {
            ElementList::iterator featureit;
            for ( featureit = camera->features.begin(); featureit != camera->features.end(); featureit++ )
            {
                Feature *feature = (Feature*)featureit->second;
                Track *track = feature->track;
                if ( track == NULL ) continue;
                
                Point *point = track->point;
                if ( point == NULL ) continue;
                
                int i = point2index[point];
                
                prefixes[j][i] = camera->node->globalPose( nodes[j] ).cast<float>();
                
                // here is where we lose the redundancy of measurements
                // only one-to-one mapping between points and nodes
                getVisibility( i, j ) = 1;
                getFeature( i, j ) = feature;
            }
        }
        
        ElementList::iterator it;
        for ( it = node->children.begin(); it != node->children.end(); it++ )
        {
            Node *child = (Node*)it->second;
            _addMeasurements( child, j );
        }
    }
    
    void Bundle::addMeasurements()
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
    
    void Bundle::fillMeasurements()
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
    
    bool Bundle::run()
    {
        bool should_update = _run();
        
        if ( should_update )
        {
            updatePoints();
            updateNodes();
        }
        
        return should_update;
    }

    void Bundle::run_str()
    {
        bool should_update = _run_str();
        
        if ( should_update )
        {
            updatePoints();
        }
    }
    
    void Bundle::run_mot()
    {
        bool should_update = _run_mot();
        
        if ( should_update )
        {
            updateNodes();
        }
    }
    
    void Bundle::getPose( int j, int i, double *aj, Sophus::SE3d &pose )
    {
        Node *node = nodes[j];
        Feature *feature = getFeature( i, j );
        Camera *camera = feature->camera;
        
        if ( upright ) {
            Eigen::Vector3d r;
            r << 0, *aj, 0;
            pose.so3() = Sophus::SO3d::exp( r );
            pose.translation() = Eigen::Map<Eigen::Vector3d>(aj+1);
        } else {
            pose = Sophus::SE3d::exp( Eigen::Map< Eigen::Matrix<double,6,1> >(aj) );
        }
        
        // pre-multiply camera's pose
        // this prefix could be pre-computed and stored
        Sophus::SE3d prefix = camera->node->globalPose( node );
        // assuming that we are one hop away
        pose = prefix * pose;
    }
    
    void proj( int j, int i, double *aj, double *bi, double *xij, void *adata )
    {
        Bundle *bundle = (Bundle*) adata;
        Feature *feature = bundle->getFeature(i,j);
        if ( feature == NULL ) {
            std::cout << "NULL feature\n";
            exit(1);
        }
        
        Camera *camera = feature->camera;

        Sophus::SE3d pose;
        bundle->getPose( j, i, aj, pose );
        Eigen::Vector3d PX = pose * Eigen::Map<Eigen::Vector3d>( bi );

        Eigen::Vector2d projpt = camera->calibration->project( project( PX ) );

        if ( std::isinf(projpt[0]) || std::isinf(projpt[1]) || std::isnan(projpt[0]) || std::isnan(projpt[1]) )
        {
            Track *track = feature->track;
            ElementList::iterator it;
            for ( it = track->features.begin(); it != track->features.end(); it++ )
            {
                Feature *feat = (Feature*)it->second;
                std::cout << feat->name << "\n";
                std::cout << feat->location << "\n";
            }
            
            std::cout << pose.log() << "\n";
            std::cout << Eigen::Map<Eigen::Vector3d>( bi ) << "\n";
            std::cout << pose * Eigen::Map<Eigen::Vector3d>( bi ) << "\n";
            std::cout << PX << "\n";
            std::cout << projpt << "\n";
            std::cout << "error: NaN or Inf\n";
            exit(0);
        }
        
        Eigen::Map<Eigen::Vector2d> xijvec( xij );
        xijvec = projpt;
    }
    
    void proj_str( int j, int i, double *bi, double *xij, void *adata )
    {
        Bundle *bundle = (Bundle*) adata;
        
        double *aj = bundle->p + bundle->cnp * j;
        
        proj( j, i, aj, bi, xij, adata );
    }

    void proj_mot( int j, int i, double *aj, double *xij, void *adata )
    {
        Bundle *bundle = (Bundle*) adata;
        
        double *bi = bundle->p + bundle->m * bundle->cnp + i * bundle->pnp;

        proj( j, i, aj, bi, xij, adata );
    }
    
    void projac( int j, int i, double *aj, double *bi, double *Aij, double *Bij, void *adata )
    {
        Bundle *bundle = (Bundle*) adata;
        Feature *feature = bundle->getFeature(i,j);
        Camera *camera = feature->camera;
        double f = camera->calibration->focal;
        
        Sophus::SE3d pose;
        bundle->getPose( j, i, aj, pose );
        Eigen::Vector3d PX = pose * Eigen::Map<Eigen::Vector3d>( bi );
        
        Eigen::Matrix<double,2,3> A;
        A <<
        PX[2], 0, -PX[0],
        0, PX[2], -PX[1];
        A *= f / (PX[2] * PX[2]);
        
        Eigen::Matrix3d rotjac;
        rotjac.col(0) = Sophus::SO3d::generator(0) * PX;
        rotjac.col(1) = Sophus::SO3d::generator(1) * PX;
        rotjac.col(2) = Sophus::SO3d::generator(2) * PX;
        
        Eigen::Matrix3d R( pose.so3().matrix() );
        
        Eigen::Matrix<double,3,6> Jmot;
        Jmot.block<3,3>(0,0) = rotjac;      // jacobians for rotation
        Jmot.block<3,3>(0,3) = Eigen::Matrix3d::Identity();  // jacobians for translation
        
        Eigen::Map< Eigen::Matrix<double,2,6> > Aijmat(Aij);
        Aijmat = A * Jmot;
        
        Eigen::Matrix3d Jstr;
        Jstr = R;    // jacobians for point
        
        Eigen::Map< Eigen::Matrix<double,2,3> > Bijmat(Bij);
        Bijmat = A * Jstr;
    }
    
    void projac_mot( int j, int i, double *aj, double *Aij, void *adata )
    {
        Bundle *bundle = (Bundle*) adata;
        double *bi = bundle->p + bundle->m * bundle->cnp + i * bundle->pnp;
        
        Feature *feature = bundle->getFeature(i,j);
        Camera *camera = feature->camera;
        double f = camera->calibration->focal;
        
        Sophus::SE3d pose;
        bundle->getPose( j, i, aj, pose );
        Eigen::Vector3d PX = pose * Eigen::Map<Eigen::Vector3d>( bi );

        Eigen::Matrix<double,2,3> A;
        A <<
        PX[2], 0, -PX[0],
        0, PX[2], -PX[1];
        A *= f / (PX[2] * PX[2]);
        
        Eigen::Matrix3d rotjac;
        rotjac.col(0) = Sophus::SO3d::generator(0) * PX;
        rotjac.col(1) = Sophus::SO3d::generator(1) * PX;
        rotjac.col(2) = Sophus::SO3d::generator(2) * PX;
        
        Eigen::Matrix<double,3,6> J;
        J.block<3,3>(0,0) = Eigen::Matrix3d::Identity();  // jacobians for translation
        J.block<3,3>(0,3) = rotjac;      // jacobians for rotation
        
        Eigen::Map< Eigen::Matrix<double,2,6> > Aijmat(Aij);
        Aijmat = A * J;
    }

    bool Bundle::_run()
    {
#ifdef USE_SBA
        int sba_verbose = 0;
        double opts[SBA_OPTSSZ] = { SBA_INIT_MU, SBA_STOP_THRESH, SBA_STOP_THRESH, SBA_STOP_THRESH, 0.0 };
        double info[SBA_INFOSZ];
        
        bool done = false;
        bool should_update = true;
        
        while ( !done ) {
            int niter = sba_motstr_levmar(n, ncon, m, mcon, vmask, p, cnp, pnp, x, covx, mnp, proj, NULL, this, itmax, sba_verbose, opts, info );
            
            done = true;
            if ( info[6] == 7 ) {
                fprintf( stderr, "error: Inf or NaN in bundle adjustment.\n" );
                should_update = false;
                break;
            } else if ( niter < 0 ) {
                fprintf( stderr, "error: SBA failed.\n" );
                should_update = false;
                break;
            } else if ( info[6] == 5 ) {
                opts[0] *= 10;
                done = false;
            }
            
            if ( verbose ) {
                cout << "average error was: " << sqrt(info[0]/o) << "\tnow: " << sqrt(info[1]/o) << "\n";
            }
        }
        
        return should_update;
#else
        Node *node = root;
        while ( node->camera == NULL ) node = (Node*)node->children.begin()->second;
        MetricPrefixBA ba( node->camera->calibration->focal, node->camera->calibration->center, prefixes );
        double *pptr = p;
	ba.mcon = mcon;
	ba.ncon = ncon;
        for ( int j = 0; j < m; j++,pptr+=6 ) ba.addCamera( wrapVector<6,double>(pptr) );
        for ( int i = 0; i < n; i++,pptr+=3 ) ba.addPoint( wrapVector<3,double>(pptr) );
        double *xptr = x;
        for ( int i = 0; i < n; i++ ) {
            for ( int j = 0; j < m; j++ ) {
                if ( getVisibility(i, j) == 0 ) continue;
                ba.addObservation( j, i, wrapVector<2,double>(xptr) );
                xptr += 2;

/*
                Matrix<2,9> jac = ba.get_jac( j, wrapVector<6,double>(p+j*6), i, wrapVector<3,double>(p+m*6+i*3) );
                Matrix<2,6> jac_mot = jac.slice(0,0,2,6);
                Matrix<2,3> jac_str = jac.slice(0,6,2,3);
	
                double my_jac_mot_data[12];
                projac_mot( j, i, p+j*6, my_jac_mot_data, this );
*/
//                cout << "new (" << i << ", " << j << ")\n" << jac_mot << "\n";
//                cout << "old (" << i << ", " << j << ")\n" << wrapMatrix<2,6>( my_jac_mot_data ) << "\n";
//
//                cout << "new str (" << i << ", " << j << ")\n" << jac_str << "\n";

            }
        }
        double old_err;
        if ( verbose ) {
            old_err = ba.getResidual();
        }
        ba.verbose = verbose;
        ba.run_all();
        pptr = p;
//        for ( int j = 0; j < m; j++,pptr+=6 ) {wrapVector<6,double>(pptr) = ba.cameras[j]; cout << wrapVector<6,double>(pptr) << "\n"; }
//        for ( int i = 0; i < n; i++,pptr+=3 ) {wrapVector<3,double>(pptr) = ba.points[i]; cout << wrapVector<3,double>(pptr) << "\n"; }
        double new_err;
        if ( verbose ) {
            new_err = ba.getResidual();
            cout << "average error was: " << sqrt(old_err) << "\tnow: " << sqrt(new_err) << "\n";
        }
//        for ( int j = 0; j < m; j++ ) cout << ba.cameras[j] << "\n";
//        for ( int i = 0; i < n; i++ ) cout << ba.points[i] << "\n";
        return true;
#endif
    }

    bool Bundle::_run_str()
    {
#ifdef USE_SBA
        int sba_verbose = 0;
        double opts[SBA_OPTSSZ] = { SBA_INIT_MU, SBA_STOP_THRESH, SBA_STOP_THRESH, SBA_STOP_THRESH, 0.0 };
        double info[SBA_INFOSZ];
        
        bool done = false;
        bool should_update = true;
        
        while ( !done ) {
            int niter = sba_str_levmar( n, ncon, m, vmask, p+m*cnp, pnp, x, covx, mnp, proj_str, NULL, this, itmax, sba_verbose, opts, info );
            
            done = true;
            if ( niter < 0 ) {
                fprintf( stderr, "error: SBA failed.\n" );
                should_update = false;
                break;
            } else if ( info[6] == 5 ) {
                opts[0] *= 10;
                done = false;
            } else if ( info[6] == 7 ) {
                fprintf( stderr, "warning: Inf or NaN in bundle adjustment.\n" );
                should_update = false;
                break;
            }
            
            if ( verbose ) {
                cout << "average error was: " << sqrt(info[0]/(o*mnp)) << "\tnow: " << sqrt(info[1]/(o*mnp)) << "\n";
            }
        }
        
        return should_update;
#else
        return false;
#endif
    }

    bool Bundle::_run_mot()
    {
#ifdef USE_SBA
        int sba_verbose = 0;
        double opts[SBA_OPTSSZ] = { SBA_INIT_MU, SBA_STOP_THRESH, SBA_STOP_THRESH, SBA_STOP_THRESH, 0.0 };
        double info[SBA_INFOSZ];
        
        bool done = false;
        bool should_update = true;
        
        while ( !done ) {
            int niter = sba_mot_levmar( n, m, mcon, vmask, p, cnp, x, covx, mnp, proj_mot, NULL, this, itmax, sba_verbose, opts, info );
               
            done = true;
            if ( niter < 0 ) {
                fprintf( stderr, "error: SBA failed.\n" );
                should_update = false;
                break;
            } else if ( info[6] == 5 ) {
                opts[0] *= 10;
                done = false;
            } else if ( info[6] == 7 ) {
                fprintf( stderr, "warning: Inf or NaN in bundle adjustment.\n" );
                should_update = false;
                break;
            }
            
            if ( verbose ) {
                cout << "average error was: " << sqrt(info[0]/o) << "\tnow: " << sqrt(info[1]/o) << "\n";
            }
        }
        
        return should_update;
#else
        Node *node = root;
        while ( node->camera == NULL ) node = (Node*)node->children.begin()->second;
        MetricPrefixBA ba( node->camera->calibration->focal, node->camera->calibration->center, prefixes );
        ba.verbose = true;
        double *pptr = p;
	ba.mcon = mcon;
	ba.ncon = ncon;
        for ( int j = 0; j < m; j++,pptr+=6 ) ba.addCamera( wrapVector<6,double>(pptr) );
        for ( int i = 0; i < n; i++,pptr+=3 ) ba.addPoint( wrapVector<3,double>(pptr) );
        double *xptr = x;
        for ( int i = 0; i < n; i++ ) {
            for ( int j = 0; j < m; j++ ) {
                if ( getVisibility(i, j) == 0 ) continue;
                ba.addObservation( j, i, wrapVector<2,double>(xptr) );
                xptr += 2;
            }
        }
        double old_err;
        if ( verbose ) {
            cout << "STARTING MOT\n";
            old_err = ba.getResidual();
        }
        ba.run_mot();
        pptr = p;
        for ( int j = 0; j < m; j++,pptr+=6 ) wrapVector<6,double>(pptr) = ba.cameras[j];
        double new_err;
        if ( verbose ) {
            //new_err = ba.getResidual();
            cout << "average error was: " << sqrt(old_err) << "\tnow: " << sqrt(new_err) << "\n";
        }
        
        return true;
#endif
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

        while ( true )
        {
            Bundle bundle( rootnode, r->upright, true );
            bool good = bundle.run();
            if ( !good ) return false;
            
            fixScale( rootnode );
            
            int numRemoved = removeOutliers( r, rootnode );
            if ( numRemoved == 0 ) break;
        }
        
        return true;
    }
}
