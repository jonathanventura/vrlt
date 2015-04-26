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
#include <FeatureMatcher/bruteforce.h>
#include <PatchTracker/tracker.h>
#include <Localizer/nnlocalizer.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <opencv2/highgui.hpp>

#include <iostream>

using namespace vrlt;

#define BUFFER_SIZE 131072

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
    ServerThread( Node *_root, Calibration *_calibration, cv::Size _imsize, int _clntSock )
    : root( _root ), imsize( _imsize ), clntSock( _clntSock )
    {
        index = new BruteForceNN;
        
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
    }

        int cnt = 0;
    
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
        
        while ( nbytesRecvd < 4 )
        {
            int recvMsgSize;
            
            // receive some data
            recvMsgSize = recv( clntSock, ptr, 4 - nbytesRecvd, 0);
            if ( recvMsgSize < 0 ) return false;
            nbytesRecvd += recvMsgSize;
            ptr += recvMsgSize;
        }
        
        std::cout << "receiving JPEG image of " << datasize << " bytes\n";
        
        char *jpegData = new char[datasize];
        
        nbytesRecvd = 0;
        ptr = jpegData;
        
        while ( nbytesRecvd < datasize )
        {
            int recvMsgSize;
            
            // receive some data
            recvMsgSize = recv( clntSock, ptr, datasize - nbytesRecvd, 0);
            if ( recvMsgSize < 0 ) return false;
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

        cnt++;
        std::stringstream name;
        name << "test" << cnt << ".bmp";
        cv::imwrite(name.str().c_str(), querycamera->image);
        
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
        double buffer[6];
        Eigen::Map< Eigen::Matrix<double,6,1> > buffervec(buffer);
        buffervec = pose.log();
        
        int nbytesSent = send( clntSock, buffer, sizeof(double)*6, 0 );
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
        bool gotHeader = recvHeader();
        
        if ( gotHeader )
        {
            for ( ; ; )
            {
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
        
        delete localizer;
        localizer = NULL;
        
        delete querynode;
        querynode = NULL;
        
        delete querycamera;
        querycamera = NULL;
        
        delete querycalibration;
        querycalibration = NULL;
    }
    
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
        DieWithError("socket() failed");
    
    /* Construct local address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
    echoServAddr.sin_family = AF_INET;                /* Internet address family */
    echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
    echoServAddr.sin_port = htons(port);              /* Local port */
    
    /* Bind to the local address */
    if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        DieWithError("bind() failed");
    
    /* Mark the socket so it will listen for incoming connections */
    if (listen(sock, 5) < 0)
        DieWithError("listen() failed");
    
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

int main( int argc, char **argv )
{
    if ( argc != 2 && argc != 3 ) {
        fprintf( stderr, "usage: %s <reconstruction> [<port>]\n", argv[0] );
        exit(1);
    }
    
    std::cout << "loading...\n";
    
    std::string pathin = std::string(argv[1]);
    int portno = 12345;
    if ( argc == 3 ) portno = atoi(argv[2]);
    
    Reconstruction r;
    r.pathPrefix = pathin;
    std::stringstream mypath;
    mypath << pathin << "/reconstruction.xml";
    XML::read( r, mypath.str() );
    
    Node *root = (Node*)r.nodes["root"];
    loadImages( pathin, root );
    XML::readDescriptors( r, root );
    
    Calibration *calibration = new Calibration;
    cv::Size imsize;
    
    // iPhone
    //calibration->focal = 1489.653430;
    // iPad
    //calibration->focal = 1179.90411;
    //calibration->center[0] = 639.500000;
    //calibration->center[1] = 359.500000;
    //imsize = cv::Size( 1280, 720 );

    // htc test full res (calibration of the requests)
    calibration->focal = 2452.49622461;
    calibration->center[0] = 1343.5;
    calibration->center[1] = 759.5;
    imsize = cv::Size( 2688, 1520 );
    
    //    imsize = imsize / 4;
    //    int level = 2;
    //    double levelScale = pow( 2., (double)-level );
    //    calibration->focal *= levelScale;
    //    calibration->center *= levelScale;
    
    int servSock = CreateTCPServerSocket(portno);
    
    std::cout << "server ready.\n";
    
#if USE_DISPATCH
    dispatch_queue_t myCustomQueue = dispatch_queue_create("com.example.MyCustomQueue", NULL);
#endif
    
    for ( ; ; )
    {
        int clntSock = AcceptTCPConnection(servSock);
        
#if USE_DISPATCH
        dispatch_async(myCustomQueue, ^{
            ServerThread *serverThread = new ServerThread( root, calibration, imsize, clntSock );
            serverThread->run();
        });
#endif
    }
    
    return 0;
}
