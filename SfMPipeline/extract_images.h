
#include <MultiView/multiview.h>

#include "threaded.h"

namespace vrlt
{
    class ImageExtractor
    {
    protected:
        cv::Size sz;
        int nfaces;
        Calibration *calibration;
        Sophus::SO3d *R;
    public:
        ImageExtractor( cv::Size &_sz, int _nfaces, Calibration *_calibration );
        virtual ~ImageExtractor();
        
        virtual void create() { }
        virtual void extractImage( cv::Mat &input_image, int face, cv::Mat &output_image ) const { };
        virtual void extractImage( cv::Mat &input_image, int face, cv::Mat &output_image, float xmul, float ymul ) const { };
        
        int getNumFaces() { return nfaces; }
        cv::Size getSize() { return sz; }
        Calibration * getCalibration() { return calibration; }
        Sophus::SO3d getRotation(int i) { return R[i]; }
        
        virtual std::string getPath( const std::string &imagedir, int index ) { return "image.jpg"; }
    };
    
    class ExtractThread : public ReconstructionThread
    {
        ImageExtractor *extractor;
        
        std::string imagedir;
        int index;
        
        Node *node;
        std::string prefix;
        
        bool delete_extractor;
    public:
        ExtractThread( ImageExtractor *_extractor, std::string _imagedir, int _index, bool _delete_extractor = false, std::string _prefix = "" );
        ~ExtractThread();
        
        void run();
        void finish( Reconstruction &r );
    };
}
