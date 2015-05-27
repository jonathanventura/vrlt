
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <LocalizerClient/client.h>

namespace vrlt {
    
    LocalizationClient::LocalizationClient( int _scale, int _shift )
    : sock( -1 ), scale( _scale ), shift( _shift )
    {

    }

    LocalizationClient::~LocalizationClient()
    {
        close( sock );
    }

    bool LocalizationClient::connectToServer( const std::string &servIP, int portno )
    {
        if ( (sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
        {
            return false;
        }
        
        struct sockaddr_in servAddr;
        memset( &servAddr, 0, sizeof(servAddr) );
        servAddr.sin_family = AF_INET;
        servAddr.sin_addr.s_addr = inet_addr(servIP.c_str());
        servAddr.sin_port = htons(portno);
        
        if ( connect( sock, (struct sockaddr *)&servAddr, sizeof(servAddr) ) < 0 )
        {
            close( sock );
            sock = -1;
            return false;
        }
        
        // send header information
        int data[2] = { scale, shift };
        if ( send( sock, data, 2*sizeof(int), 0 ) != 2*sizeof(int) )
        {
            close( sock );
            sock = -1;
            return false;
        }
        
        return true;
    }

    bool LocalizationClient::sendImage( int nbytes, unsigned char *bytes )
    {
        if ( send( sock, &nbytes, sizeof(int), 0 ) != sizeof(int) )
        {
            close( sock );
            sock = -1;
            return false;
        }
        
        if ( send( sock, bytes, nbytes, 0 ) != nbytes )
        {
            close( sock );
            sock = -1;
            return false;
        }
        
        return true;
    }

    bool LocalizationClient::recvPose( double *posedata )
    {
        unsigned char *ptr = (unsigned char *)posedata;
        int totalrecvd = 0;
        while ( totalrecvd < 12*sizeof(double) )
        {
            int bytesrecvd = recv( sock, ptr, 12*sizeof(double) - totalrecvd, 0 );
            if ( bytesrecvd < 0 )
            {
                close( sock );
                sock = -1;
                return false;
            }
            
            ptr += bytesrecvd;
            totalrecvd += bytesrecvd;
        }
        
        return true;
    }
}
