/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: server.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>
#include <FeatureExtraction/features.h>
//#include <FeatureMatcher/bruteforce.h>
#include <FeatureMatcher/approxnn.h>
#include <PatchTracker/tracker.h>
#include <Localizer/nnlocalizer.h>

#include <GeographicLib/UTMUPS.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <map>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <iostream>

#if USE_DISPATCH
#include <dispatch/dispatch.h>
#endif

using namespace vrlt;

#define BUFFER_SIZE 131072
#define KALMAN

//#define AVERAGING
static const int WINDOWSIZE = 5;


// from http://stackoverflow.com/questions/2079912/simpler-way-to-create-a-c-memorystream-from-char-size-t-without-copying-t
class membuf : public std::basic_streambuf<char>
{
public:
    membuf(char* p, size_t n) {
        setg( p, p, p + n );
    }
};

class ServerThread
{
public:
    ServerThread(const Reconstruction *_r, Node *_root, Calibration *_calibration, cv::Size _imsize, int _clntSock )
    : r(_r), root( _root ), imsize( _imsize ), clntSock( _clntSock )
    {
        index = new ApproxNN;
        
        localizer = new NNLocalizer( root, index );
        localizer->verbose = true;
        //        localizer->tracker->firstlevel = 3;
        //        localizer->tracker->lastlevel = 1;
        localizer->tracker->minnumpoints = 200;
        localizer->thresh = 0.006 * imsize.width / _calibration->focal;
        //        localizer->thresh *= 2.;
        
        querynode = new Node;
        querynode->name = "querynode";
        
        querycamera = new Camera;
        querycamera->name = "querycamera";
        querycamera->node = querynode;
        querynode->camera = querycamera;
        
        querycalibration = new Calibration;
        querycalibration->name = "querycalibration";
        querycalibration->focal = _calibration->focal;
        querycalibration->center = _calibration->center;
        querycamera->calibration = querycalibration;
        
        buffer = new char[BUFFER_SIZE];

        imageCnt = 0;

#ifdef KALMAN
        LocalizationKf = new cv::KalmanFilter(4,2,0);
        LocalizationKf->transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0,
                                                                     0,1,0,1,
                                                                     0,0,1,0,
                                                                     0,0,0,1);

        cv::setIdentity(LocalizationKf->measurementMatrix);
        cv::setIdentity(LocalizationKf->processNoiseCov, cv::Scalar::all(1e-6));
        cv::setIdentity(LocalizationKf->measurementNoiseCov, cv::Scalar::all(1e-6));
        LocalizationKf->measurementNoiseCov.at<float>(0,0) = 0.01;
        LocalizationKf->measurementNoiseCov.at<float>(1,1) = 0.01;
        cv::setIdentity(LocalizationKf->errorCovPost, cv::Scalar::all(.1));
        LocalizationKf->errorCovPost.at<float>(0,0) = 0.1471;//-0.1471;
        LocalizationKf->errorCovPost.at<float>(1,1) = 0.6012;//-0.6012;


        AltitudeKf = new cv::KalmanFilter(1,1);
        AltitudeKf->transitionMatrix = (cv::Mat_<float>(1, 1) << 1);
        AltitudeKf->statePre.at<float>(0) = 1.5;
        cv::setIdentity(AltitudeKf->measurementMatrix);
        cv::setIdentity(AltitudeKf->processNoiseCov, cv::Scalar::all(1e-6));
        cv::setIdentity(AltitudeKf->measurementNoiseCov, cv::Scalar::all(0.0227));
        cv::setIdentity(AltitudeKf->errorCovPost, cv::Scalar::all(0.3771));//1.4std
#endif
    }
    
    ~ServerThread()
    {
        delete [] buffer;
    }
    
    unsigned char * unpackBytes( unsigned char *ptr, char *bufferptr, int recvMsgSize )
    {
        for ( int i = 0; i < recvMsgSize; i++,bufferptr++ )
        {
            unsigned char val = *bufferptr;
            if ( bpp == 8 )
            {
                *ptr = val;
                ptr++;
            } else if ( bpp == 4 ) {
                *ptr = (val & 0xF0);
                ptr++;
                *ptr = (val & 0x0F) << 4;
                ptr++;
            } else if ( bpp == 2 ) {
                *ptr = (val & 0xD0);
                ptr++;
                *ptr = (val & 0x30) << 2;
                ptr++;
                *ptr = (val & 0x0D) << 4;
                ptr++;
                *ptr = (val & 0x03) << 6;
                ptr++;
            } else if ( bpp == 1 ) {
                *ptr = (val & 0x80);
                ptr++;
                *ptr = (val & 0x40) << 1;
                ptr++;
                *ptr = (val & 0x20) << 2;
                ptr++;
                *ptr = (val & 0x10) << 3;
                ptr++;
                *ptr = (val & 0x08) << 4;
                ptr++;
                *ptr = (val & 0x04) << 5;
                ptr++;
                *ptr = (val & 0x02) << 6;
                ptr++;
                *ptr = (val & 0x01) << 7;
                ptr++;
            }
        }
        
        return ptr;
    }
    
    bool waitForImage()
    {
        int nbytesRecvd = 0;
        char *ptr = NULL;
        
        int datasize = 0;
        ptr = (char*)&datasize;

        int timeout = 0;
        
        while ( nbytesRecvd < 4 )
        {
            int recvMsgSize;
            
            // receive some data
            recvMsgSize = recv( clntSock, ptr, 4 - nbytesRecvd, 0);
            if ( recvMsgSize < 0 || timeout > 200 ) return false;
            if( recvMsgSize == 0) timeout++;
            nbytesRecvd += recvMsgSize;
            ptr += recvMsgSize;
        }
        
        std::cout << "receiving JPEG image of " << datasize << " bytes\n";
        
        char *jpegData = new char[datasize];
        
        nbytesRecvd = 0;
        ptr = jpegData;
        timeout = 0;
        
        while ( nbytesRecvd < datasize )
        {
            int recvMsgSize;
            
            // receive some data
            recvMsgSize = recv( clntSock, ptr, datasize - nbytesRecvd, 0);
            if ( recvMsgSize < 0 || timeout > 200 ) return false;
            if( recvMsgSize == 0) timeout++;
            nbytesRecvd += recvMsgSize;
            ptr += recvMsgSize;
        }
        
        //        cout << "writing JPEG data to file\n";
        //
        //        FILE *f = fopen( "/tmp/temp.jpg", "w" );
        //        fwrite( jpegData, 1, datasize, f );
        //        fclose( f );
        //
        //        querycamera->image = img_load( "/tmp/temp.jpg" );
        //
        //        cout << "read JPEG image from file\n";
        
        //        membuf mb( jpegData, datasize );
        //        std::istream is( &mb );
        cv::Mat jpegDataMat( cv::Size(datasize,1), CV_8UC1, jpegData );
        querycamera->image = cv::imdecode( jpegDataMat, cv::IMREAD_UNCHANGED );

        //imageCnt++;
        //std::stringstream name;
        //name << imageCnt << ".bmp";
        //cv::imwrite(name.str().c_str(), querycamera->image);
        
        delete [] jpegData;
        
        /*
         unsigned char *ptr = querycamera->image.data();
         
         while ( nbytesRecvd < imsize.x * imsize.y * bpp / 8 )
         {
         int recvMsgSize;
         
         // receive some data
         recvMsgSize = recv( clntSock, buffer, BUFFER_SIZE, 0);
         if ( recvMsgSize < 0 ) return false;
         nbytesRecvd += recvMsgSize;
         
         ptr = unpackBytes( ptr, buffer, recvMsgSize );
         }
         
         cout << "read a whole image!\n";
         */
        
        return true;
    }
    
    bool sendPose( Sophus::SE3d &pose )
    {
        double buffer[12];
        //Eigen::Map< Eigen::Matrix<double,6,1> > buffervec(buffer);
        //buffervec = pose.log();

        //std::cout << "sophus translation x:" << pose.translation()[0] << "  y: " << pose.translation()[1] << " z: " << pose.translation()[2] << std::endl;
        bool success = false;
        for ( int i = 0; i < 6; i++ ) if ( pose.log()[i] != 0 ) {
           success = true;
           break;
        }


        if(success) {

            Eigen::Matrix3d rot;
            rot <<  0,  0,  -1,
                  -1,  0,  0,
                  0, 1,  0; //
            Sophus::SO3d rotAr2Pointing(rot);

            Eigen::Matrix3d t;
            t <<  1,  0,  0,
                  0,  0,  -1,
                  0, 1,  0; //
            Sophus::SO3d vrlt2opengl(t);

            Sophus::SO3d openglRotation = vrlt2opengl.inverse() * pose.so3().inverse() * rotAr2Pointing * vrlt2opengl;

            //according to Hartley and Zissermann
            //P = [R|t]
            //P = [R|-Rc]
            //t = -Rc
            //c = -R't
            Eigen::Vector3d center = -(pose.so3().inverse()*pose.translation());



#ifdef AVERAGING
    if(avgwindowX.size() < WINDOWSIZE) {
        avgwindowX.push_back(center[0]);
        avgwindowY.push_back(center[2]);
        avgwindowZ.push_back(center[1]);
    } else {
        avgwindowX.at(imageCnt % WINDOWSIZE) = center[0];
        avgwindowY.at(imageCnt % WINDOWSIZE) = center[2];
        avgwindowZ.at(imageCnt % WINDOWSIZE) = center[1];
    }

    center[0] = center[2] = center[1] = 0.0;

    for (int i = 0; i < avgwindowX.size(); ++i) {
        center[0]+= avgwindowX.at(i);
        center[2]+= avgwindowY.at(i);
        center[1]+= avgwindowZ.at(i);
    }

    center[0]/= avgwindowX.size();
    center[2]/= avgwindowY.size();
    center[1]/= avgwindowZ.size();
#endif



#ifdef KALMAN

            if(imageCnt == 0){
                LocalizationKf->statePre.at<float>(0) = center[0];
                LocalizationKf->statePre.at<float>(1) = center[2];
                LocalizationKf->statePre.at<float>(2) = 0;
                LocalizationKf->statePre.at<float>(3) = 0;
            }
            LocalizationKf->predict();

            cv::Mat_<float> locationMeasurement(2,1); locationMeasurement.setTo(cv::Scalar(0));
            locationMeasurement(0) = center[0];
            locationMeasurement(1) = center[2];

            cv::Mat estimate = LocalizationKf->correct(locationMeasurement);
            center[0] = estimate.at<float>(0); center[2] = estimate.at<float>(1);


            AltitudeKf->predict();

            cv::Mat_<float> heightMeasurement(1,1);
            heightMeasurement(0) = center[1];
            cv::Mat heightEstimate = AltitudeKf->correct(heightMeasurement);
            center[1] = heightEstimate.at<float>(0);
#endif

       imageCnt++;

       double lat, lon;
       GeographicLib::UTMUPS::Reverse(r->utmZone, r->utmNorth,
                                      r->utmCenterEast + center[0], r->utmCenterNorth + center[2],
                                      lat, lon);


        //passing the matrix like that is correct...
        buffer[0] = center[0]/*lat*/; buffer[1] = center[2]/*lon*/; buffer[2] = -center[1];
        buffer[3] = openglRotation.matrix()(0,0); buffer[4] = openglRotation.matrix()(1,0); buffer[5] = openglRotation.matrix()(2,0);
        buffer[6] = openglRotation.matrix()(0,1); buffer[7] = openglRotation.matrix()(1,1); buffer[8] = openglRotation.matrix()(2,1);
        buffer[9] = openglRotation.matrix()(0,2); buffer[10] = openglRotation.matrix()(1,2); buffer[11] = openglRotation.matrix()(2,2);

        }
        else {
            for ( int i = 0; i < 12; i++ ) buffer[i] = 0.;
        }

        for (int var = 0; var < 12; ++var) {
            printf("%f\n",buffer[var]);
            //std::cout << buffer[var] << std::endl;
        }
        
        int nbytesSent = send( clntSock, buffer, sizeof(double)*12, 0 );
        if ( nbytesSent < 0 ) return false;
        
        return true;
    }
    
    bool recvHeader()
    {
        int data[2];
        
        if ( recv( clntSock, data, 2*sizeof(int), 0 ) != 2*sizeof(int) ) return false;
        
        /*
         int scale = data[0];
         int shift = data[1];
         */
        
        int scale = 1;
        int shift = 0;
        
        if ( scale == 2 )
        {
            localizer->tracker->firstlevel -= 1;
            localizer->tracker->lastlevel -= 1;
        }
        if ( localizer->tracker->firstlevel < 0 ) localizer->tracker->firstlevel = 0;
        if ( localizer->tracker->lastlevel < 0 ) localizer->tracker->lastlevel = 0;
        
        imsize.width /= scale;
        imsize.height /= scale;
        querycamera->image.create( imsize, CV_8UC1 );
        
        querycalibration->focal /= scale;
        querycalibration->center /= scale;
        
        localizer->thresh /= scale;
        
        bpp = 8 - shift;
        
        return true;
    }
    
    void run()
    {
        std::cout << "read header\n";
        bool gotHeader = recvHeader();
        std::cout << "recived header\n";
        
        if ( gotHeader )
        {
            for ( ; ; )
            {
                std::cout << "wait for image\n";
                bool good = waitForImage();
                if ( !good ) break;
                
                //                bool good = true;
                //                static int mynum = 1;
                //                stringstream mypath;
                //                mypath << "kirbyquery1/image" << mynum++ << ".jpg";
                //                if ( mynum > 58 ) break;
                //                Image<byte> myim = img_load( mypath.str() );
                //                querycamera->image.copy_from( myim );
                //                querycamera->name = mypath.str();
                //                querynode->name = mypath.str();
                
                //                stringstream path;
                //                static int count = 0;
                //                path << "Output/image" << count++ << ".jpg";
                //                img_save( querycamera->image, path.str(), ImageType::JPEG );
                
                std::vector<Feature*> features;
                extractSIFT( querycamera->image, features );
                
                for ( int i = 0; i < features.size(); i++ )
                {
                    std::stringstream name;
                    name << "feature" << i << "\n";
                    features[i]->name = name.str();
                    features[i]->camera = querycamera;
                    querycamera->features[features[i]->name] = features[i];
                }
                
                querycamera->pyramid.resize( querycamera->image.size() );
                querycamera->pyramid.copy_from( querycamera->image );
                
                
                bool success = localizer->localize( querycamera );
                
                
                //bool success = false;
                
                Sophus::SE3d pose;
                if ( success ) pose = querynode->pose;
                
                std::cout << "sending pose...\n";
                good = sendPose( pose );
                if ( !good ) break;
                
                if ( good ) std::cout << "pose: " << pose.log() << "\n";
                
                for ( int i = 0; i < features.size(); i++ )
                {
                    delete features[i];
                }
                querycamera->features.clear();
            }
        }
        
        std::cout << "closing connection...\n";
        
        close( clntSock );
        
        std::cout << "delete localizer \n";
        //if(localizer)  //todo here is an issue the localizer is not deleted correctly
        delete localizer;
        localizer = NULL;
        
        std::cout << "delete querynode \n";
        delete querynode;
        querynode = NULL;
        
        std::cout << "delete querycamera \n";
        delete querycamera;
        querycamera = NULL;

        std::cout << "delete querycalibration \n";
        delete querycalibration;
        querycalibration = NULL;

#ifdef KALMAN
        delete LocalizationKf;
        LocalizationKf = NULL;
        delete AltitudeKf;
        AltitudeKf = NULL;
#endif
    }
    
    const Reconstruction *r;
    Node *root;
    cv::Size imsize;
    
    Node *querynode;
    Camera *querycamera;
    Calibration *querycalibration;
    
    NN *index;
    Localizer *localizer;
    
    int clntSock;
    char *buffer;
    
    int bpp;

    int imageCnt;

#ifdef AVERAGING
    std::vector<double> avgwindowX;
    std::vector<double> avgwindowY;
    std::vector<double> avgwindowZ;
#endif

#ifdef KALMAN
    cv::KalmanFilter *LocalizationKf;
    cv::KalmanFilter *AltitudeKf;
#endif

};

void DieWithError( const char *msg )
{
    fprintf( stderr, "error: %s\n", msg );
    exit(1);
}

int CreateTCPServerSocket(unsigned short port)
{
    int sock;                        /* socket to create */
    struct sockaddr_in echoServAddr; /* Local address */
    
    /* Create socket for incoming connections */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    {
        printf("socket() failed");
        return -1;
    }
    
    /* Construct local address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
    echoServAddr.sin_family = AF_INET;                /* Internet address family */
    echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
    echoServAddr.sin_port = htons(port);              /* Local port */
    
    /* Bind to the local address */
    if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
    {
        printf("bind() failed");
        return -1;
    }
    
    /* Mark the socket so it will listen for incoming connections */
    if (listen(sock, 5) < 0)
    {
        printf("listen() failed");
        return -1;
    }
    
    return sock;
}

int AcceptTCPConnection(int servSock)
{
    int clntSock;                    /* Socket descriptor for client */
    struct sockaddr_in echoClntAddr; /* Client address */
    unsigned int clntLen;            /* Length of client address data structure */
    
    /* Set the size of the in-out parameter */
    clntLen = sizeof(echoClntAddr);
    
    /* Wait for a client to connect */
    if ((clntSock = accept(servSock, (struct sockaddr *) &echoClntAddr,
                           &clntLen)) < 0)
        DieWithError("accept() failed");
    
    /* clntSock is connected to a client! */
    
    printf("Handling client %s\n", inet_ntoa(echoClntAddr.sin_addr));
    
    return clntSock;
}

void loadImages( std::string prefix, Node *node )
{
    if ( node->camera != NULL ) {
        std::stringstream path;
        path << prefix << "/" << node->camera->path;
        node->camera->image = cv::imread( path.str(), cv::IMREAD_GRAYSCALE );
        node->camera->pyramid = ImagePyramid( node->camera );
    }
    
    ElementList::iterator it;
    for ( it = node->children.begin(); it != node->children.end(); it++ )
    {
        Node *child = (Node *)it->second;
        loadImages( prefix, child );
    }
}

bool parseConfig(std::map<std::string, std::string> &options, const char* path) {

    std::string id, eq, val;
    std::ifstream cfgfile (path, std::ifstream::in);

    while(cfgfile >> id >> eq >> val)
    {
      if (id[0] == '#') continue;  // skip comments
      if (eq != "=") {
          cfgfile.close();
          return false;
      }

      options[id] = val;
    }

    cfgfile.close();
    return true;
}



int main( int argc, char **argv )
{
    if ( argc != 3 && argc != 4 ) {
        fprintf( stderr, "usage: %s <reconstruction> <config> [<port>]\n", argv[0] );
        exit(1);
    }
    
    std::cout << "loading...\n";
    
    std::string pathin = std::string(argv[1]);
    int portno = 12345;
    if ( argc == 4 ) portno = atoi(argv[3]);
    
    Reconstruction r;
    r.pathPrefix = pathin;
    std::stringstream mypath;
    mypath << pathin << "/reconstruction.xml";
    XML::read( r, mypath.str() );
    
    Node *root = (Node*)r.nodes["root"];
    loadImages( pathin, root );
    XML::readDescriptors( r, root );
    
    std::map<std::string, std::string> options;
    if(!parseConfig(options, argv[2])) {
        fprintf( stderr, "config file corrupted\n" );
        exit(1);
    }

    Calibration *calibration = new Calibration;
    cv::Size imsize;

    calibration->focal = std::stod(options.find("calibrationFocalLength")->second);
    calibration->center[0] = std::stod(options.find("calibrationPrincipalPointX")->second);
    calibration->center[1] = std::stod(options.find("calibrationPrincipalPointY")->second);
    imsize = cv::Size( std::stoi(options.find("imageSizeX")->second), std::stoi(options.find("imageSizeY")->second) );

    //    imsize = imsize / 4;
    //    int level = 2;
    //    double levelScale = pow( 2., (double)-level );
    //    calibration->focal *= levelScale;
    //    calibration->center *= levelScale;
    
    int servSock = -1;
    for ( int i = 0; i < 100; i++ )
    {
        servSock = CreateTCPServerSocket(portno+i);
        if ( servSock != -1 ) {
            printf("opened port %d\n",portno+i);
            break;
        }
    }
    
    std::cout << "server ready.\n";

    printf("%i %i %f %f\n", r.utmZone, r.utmNorth, r.utmCenterNorth, r.utmCenterEast);
    
#if USE_DISPATCH
    dispatch_queue_t myCustomQueue = dispatch_queue_create("com.example.MyCustomQueue", NULL);
#endif
    
    for ( ; ; )
    {
        int clntSock = AcceptTCPConnection(servSock);
        
#if USE_DISPATCH
        dispatch_async(myCustomQueue, ^{
            ServerThread *serverThread = new ServerThread(&r, root, calibration, imsize, clntSock );
            serverThread->run();
        });
#endif
    }
    
    return 0;
}
