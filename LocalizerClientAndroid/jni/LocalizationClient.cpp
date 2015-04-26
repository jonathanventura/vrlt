#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include<jni.h>

#include "LocalizationClient.h"

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnCreate(
		JNIEnv *env, jobject obj);
JNIEXPORT jboolean JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnStart(
		JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnPause(
		JNIEnv *env, jobject obj);
JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnDestroy(
		JNIEnv *env, jobject obj);

JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeFillImageBuffer(
		JNIEnv *env, jobject obj, jlong matptr);
JNIEXPORT jboolean JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeTransmitImageBuffer(
		JNIEnv *env, jobject obj);
JNIEXPORT jdoubleArray JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeReceiveServerPose(
		JNIEnv *env, jobject obj);
}

namespace vrlt {

JavaVM* javaVirtualMachine = nullptr;
jobject javaCommunicationWraperObj = nullptr;
LocalizationClient *localizationCleint = nullptr;

LocalizationClient::LocalizationClient(int _scale, int _shift) :
		sock(-1), scale(_scale), shift(_shift) {

}

LocalizationClient::~LocalizationClient() {
	closeConnection();
}

void LocalizationClient::closeConnection() {
	close(sock);
}

bool LocalizationClient::connectToServer(const std::string &servIP,
		int portno) {
	if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		return false;
	}

	struct sockaddr_in servAddr;
	memset(&servAddr, 0, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(servIP.c_str());
	servAddr.sin_port = htons(portno);

	if (connect(sock, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) {
		close(sock);
		sock = -1;
		return false;
	}

	// send header information
	int data[2] = { scale, shift };
	if (send(sock, data, 2 * sizeof(int), 0) != 2 * sizeof(int)) {
		close(sock);
		sock = -1;
		return false;
	}

	return true;
}

void LocalizationClient::getJpgBuffer(cv::Mat &grayMat, int compression) {

	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(compression);

	cv::imencode(".jpg", grayMat, transmissionBuffer, compression_params);
	LOGI("jpg buffer size %i", transmissionBuffer.size());
}

bool LocalizationClient::sendImage() {
	int nbytes = transmissionBuffer.size();
	unsigned char *bytes = transmissionBuffer.data();
	//int nbytes, unsigned char *bytes
	if (send(sock, &nbytes, sizeof(int), 0) != sizeof(int)) {
		close(sock);
		sock = -1;
		return false;
	}

	if (send(sock, bytes, nbytes, 0) != nbytes) {
		close(sock);
		sock = -1;
		return false;
	}

	return true;
}

bool LocalizationClient::recvPose(double *posedata) {
	unsigned char *ptr = (unsigned char *) posedata;
	int totalrecvd = 0;
	while (totalrecvd < 6 * sizeof(double)) {
		int bytesrecvd = recv(sock, ptr, 6 * sizeof(double) - totalrecvd, 0);
		if (bytesrecvd < 0) {
			close(sock);
			sock = -1;
			return false;
		}

		ptr += bytesrecvd;
		totalrecvd += bytesrecvd;
	}
	return true;
}

} //namespace vrlt

JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnCreate(
		JNIEnv * env, jobject obj) {
	//create jni cache
	LOGI("native on create");
	if (!vrlt::javaVirtualMachine) {
		env->GetJavaVM(&vrlt::javaVirtualMachine);
	}

	if (vrlt::javaCommunicationWraperObj != nullptr) {
		env->DeleteGlobalRef(vrlt::javaCommunicationWraperObj);
		vrlt::javaCommunicationWraperObj = nullptr;
	}
	vrlt::javaCommunicationWraperObj = env->NewGlobalRef(obj);

	//set scale and shift hardcoded:-(
	vrlt::localizationCleint = new vrlt::LocalizationClient(1, 0);
}

JNIEXPORT jboolean JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnStart(
		JNIEnv *env, jobject obj) {
	LOGI("native on start");
	//ip and port hardcoded:-(
	const std::string ip("192.168.1.5");
	const int port = 12345;
	if (vrlt::localizationCleint != nullptr)
		return vrlt::localizationCleint->connectToServer(ip, port);
	return false;
}

JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnPause(
		JNIEnv *env, jobject obj) {
	if (vrlt::localizationCleint != nullptr)
		vrlt::localizationCleint->closeConnection();
}

JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeOnDestroy(
		JNIEnv *env, jobject obj) {
	if (vrlt::localizationCleint != nullptr)
		delete vrlt::localizationCleint;
}

JNIEXPORT void JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeFillImageBuffer(
		JNIEnv *env, jobject obj, jlong matptr) {
	LOGI("native fill image buffer");
	if (vrlt::localizationCleint != nullptr)
		vrlt::localizationCleint->getJpgBuffer(*((cv::Mat*) matptr), 20);
}

JNIEXPORT jboolean JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeTransmitImageBuffer(
		JNIEnv *env, jobject obj) {
	LOGI("native transmit image buffer");
	if (vrlt::localizationCleint != nullptr)
		return vrlt::localizationCleint->sendImage();
	return false;
}

JNIEXPORT jdoubleArray JNICALL Java_org_opencv_samples_facedetect_NativeCommunicationWraper_nativeReceiveServerPose(
		JNIEnv *env, jobject obj) {
	LOGI("native receive server pose");
	if (vrlt::localizationCleint == nullptr)
		return NULL;

	double pose[] = { 0., 0., 0., 0., 0., 0. };
	vrlt::localizationCleint->recvPose(pose);

	jdoubleArray result = env->NewDoubleArray(6);
	if (result == NULL) {
		return NULL; /* out of memory error thrown */
	}
	env->SetDoubleArrayRegion(result, 0, 6, pose);
	return result;
}

