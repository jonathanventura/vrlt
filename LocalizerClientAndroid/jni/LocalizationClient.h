/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: client.h
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#ifndef CLIENT_H
#define CLIENT_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <android/log.h>

#define  LOG_TAG    "LocalizationClient"
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

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
        void closeConnection();
        bool sendImage();
        bool recvPose( double *posedata );
        void getJpgBuffer(cv::Mat &grayMat, int compression);
    protected:

        int sock;
        int scale;
        int shift;

        std::vector<uchar> transmissionBuffer;
    };

/**
 * @}
 */

}

#endif
