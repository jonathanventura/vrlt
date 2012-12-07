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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <cvd/image_io.h>
#include <cvd/draw.h>
#include <cvd/timer.h>
#include <cvd/thread.h>

using namespace vrlt;
using namespace std;
using namespace CVD;
using namespace TooN;

#define BUFFER_SIZE 131072

// from http://stackoverflow.com/questions/2079912/simpler-way-to-create-a-c-memorystream-from-char-size-t-without-copying-t
class membuf : public basic_streambuf<char>
{
public:
    membuf(char* p, size_t n) {
        setg( p, p, p + n );
    }
};

class ServerThread : public Thread
{
public:
    ServerThread( Reconstruction &_r, Calibration *_calibration, ImageRef _imsize, int _clntSock )
    : r( _r ), imsize( _imsize ), clntSock( _clntSock )
    {
        root = (Node*) r.nodes["root"];
        
        index = new BruteForceNN;
        
        localizer = new NNLocalizer( root, index );
        localizer->verbose = true;
        //        localizer->tracker->firstlevel = 3;
        //        localizer->tracker->lastlevel = 1;
        localizer->tracker->minnumpoints = 200;
        localizer->thresh = 0.006 * imsize.x / _calibration->focal;
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
        
        cout << "receiving JPEG image of " << datasize << " bytes\n";
        
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
        
        membuf mb( jpegData, datasize );
        istream is( &mb );
        querycamera->image = img_load( is );
        
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
    
    bool sendPose( SE3<> &pose )
    {
        double buffer[6];
        wrapVector<6>(buffer) = pose.ln();
        
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
        
        imsize /= scale;
        querycamera->image.resize( imsize );
        
        querycalibration->focal /= scale;
        querycalibration->center /= scale;
        
        localizer->thresh /= scale;
        
        bpp = 8 - shift;
        
        return true;
    }
    
    void run()
    {
        SimpleTimer featureTimer( "extract features", 1 );
        SimpleTimer localizeTimer( "localize", 1 );
        
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
                
                featureTimer.click();
                vector<Feature*> features;
                extractSIFT( querycamera->image, features );
                featureTimer.click();
                
                for ( int i = 0; i < features.size(); i++ )
                {
                    stringstream name;
                    name << "feature" << i << "\n";
                    features[i]->name = name.str();
                    features[i]->camera = querycamera;
                    querycamera->features[features[i]->name] = features[i];
                }
                
                querycamera->pyramid.resize( querycamera->image.size() );
                querycamera->pyramid.copy_from( querycamera->image );
                
                
                localizeTimer.click();
                bool success = localizer->localize( querycamera );
                localizeTimer.click();
                
                
                //bool success = false;
                
                SE3<> pose;
                if ( success ) pose = querynode->pose;
                
                good = sendPose( pose );
                if ( !good ) break;
                
                if ( good ) cout << "pose: " << pose.ln() << "\n";
                
                for ( int i = 0; i < features.size(); i++ )
                {
                    delete features[i];
                }
                querycamera->features.clear();
            }
        }
        
        cout << "closing connection...\n";
        
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
    
    Reconstruction &r;
    Node *root;
    ImageRef imsize;
    
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

void loadImages( string prefix, Node *node )
{
    if ( node->camera != NULL ) {
        stringstream path;
        path << prefix << "/" << node->camera->path;
        node->camera->image = img_load( path.str() );
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
    
    cout << "loading...\n";
    
    string pathin = string(argv[1]);
    int portno = 12345;
    if ( argc == 3 ) portno = atoi(argv[2]);
    
    Reconstruction r;
    r.pathPrefix = pathin;
    stringstream mypath;
    mypath << pathin << "/reconstruction.xml";
    XML::read( r, mypath.str() );
    
    Node *root = (Node*)r.nodes["root"];
    loadImages( pathin, root );
    XML::readDescriptors( r, root );
    
    Calibration *calibration = new Calibration;
    ImageRef imsize;
    
    // iPhone
    //calibration->focal = 1489.653430;
    // iPad
    calibration->focal = 1179.90411;
    calibration->center[0] = 639.500000;
    calibration->center[1] = 359.500000;
    imsize = ImageRef( 1280, 720 );
    
    //    imsize = imsize / 4;
    //    int level = 2;
    //    double levelScale = pow( 2., (double)-level );
    //    calibration->focal *= levelScale;
    //    calibration->center *= levelScale;
    
    int servSock = CreateTCPServerSocket(portno);
    
    cout << "server ready.\n";
    
    for ( ; ; )
    {
        int clntSock = AcceptTCPConnection(servSock);
        
        ServerThread *serverThread = new ServerThread( r, calibration, imsize, clntSock );
        serverThread->start();
    }
    
    return 0;
}