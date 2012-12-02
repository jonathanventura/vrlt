/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: client.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef CLIENT_H
#define CLIENT_H

#include <string>

namespace vrlt {

/**
 * \addtogroup Localizer
 * @{
 */

    class LocalizationClient
    {
    public:
        LocalizationClient( int _scale, int _shift );
        ~LocalizationClient();
        
        bool connectToServer( const std::string &servIP, int portno );
        bool sendImage( int nbytes, unsigned char *bytes );
        bool recvPose( double *posedata );
    protected:
        int sock;
        int scale;
        int shift;
    };

/**
 * @}
 */
    
}

#endif
