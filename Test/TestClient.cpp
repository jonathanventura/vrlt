
#include "LocalizerClient/client.h"
#include <opencv2/opencv.hpp>

int main( int argc, char **argv )
{
    if ( argc != 2 ) {
        fprintf( stderr, "usage: %s <seq> \n", argv[0] );
        exit(1);
    }

    vrlt::LocalizationClient loc(1,0);
    if (!loc.connectToServer("127.0.0.1",12345)){
        fprintf( stderr, "connection failed \n" );
        exit(1);
    }

    //std::cout << "connected...\n";

    int imgcnt = 1;
    std::stringstream stream;
    stream << argv[1] << "/" << imgcnt << ".bmp";
    cv::Mat img = cv::imread(stream.str().c_str(),cv::IMREAD_GRAYSCALE);

    std::vector<uchar> transmissionBuffer;
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(0);

    std::cout << "pose = [";

    double pose[12] = { 0., 0., 0., 0., 0., 0.,
                        0., 0., 0., 0., 0., 0.};
    while (img.data) {
        cv::imencode(".jpg", img, transmissionBuffer, compression_params);
        loc.sendImage(transmissionBuffer.size(), (unsigned char *)transmissionBuffer.data());
        loc.recvPose(pose);

        bool success = false;
        for ( int i = 0; i < 12; i++ ) if ( pose[i] != 0 ) {
            success = true;
            break;
        }
        if(success) printf("%.10f, %.10f, %.10f; ",pose[0], pose[1], pose[2]);

        stream.str("");
        imgcnt++;
        stream << argv[1] << "/" << imgcnt << ".bmp";

        img = cv::imread(stream.str().c_str(),cv::IMREAD_GRAYSCALE);
    }
    std::cout << " ];";

}
