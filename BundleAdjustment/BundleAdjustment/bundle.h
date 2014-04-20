
#include <MultiView/multiview.h>

namespace vrlt
{
    class Bundle
    {
    public:
        Bundle( Node *_root, bool _upright = false, bool _verbose = false );
        Bundle( Node *_root, const ElementList &_fixedNodes, const ElementList &_fixedPoints, bool _upright = false, bool _verbose = false );
        ~Bundle();
        
        bool verbose;
        int itmax;
        
        bool run();
        void run_str();
        void run_mot();
    protected:
        Node *root;
        bool upright;
        
        ElementList fixedNodes;
        ElementList fixedPoints;
        
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

        std::map< int, std::map< int, Sophus::SE3f > > prefixes;
        
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
        
        void getPose( int j, int i, double *aj, Sophus::SE3d &pose );
        
        friend void proj( int j, int i, double *aj, double *bi, double *xij, void *adata );
        friend void proj_str( int j, int i, double *bi, double *xij, void *adata );
        friend void proj_mot( int j, int i, double *aj, double *xij, void *adata );
        friend void projac_mot( int j, int i, double *aj, double *Aij, void *adata );
        friend void projac( int j, int i, double *aj, double *bi, double *Aij, double *Bij, void *adata );
    };
    
    void fixScale( Node *root );
    bool runBundle( Reconstruction *r );
}
