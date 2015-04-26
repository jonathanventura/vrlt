package org.opencv.samples.facedetect;

import org.opencv.core.Mat;

public class NativeCommunicationWraper {
  private boolean mConnected;
  private String mIp = "";
  private int mPort = 0;

  public NativeCommunicationWraper() {
    nativeOnCreate();
    mConnected = false;
  }

  public void connect(String ip, int port) {
    mIp = ip;
    mPort = port;
    mConnected = nativeConnect(ip, port);
  }

  public void stop() {
    nativeOnPause();
  }

  public void fillImageBuffer(Mat imageGray) {
    nativeFillImageBuffer(imageGray.getNativeObjAddr());
  }

  public boolean transmitImageBuffer() {
    //reconnect if connection is lost
    if(!mConnected) {
      mConnected = nativeConnect(mIp, mPort);
      if(!mConnected) {
        return false;
      }
    }
    return nativeTransmitImageBuffer();
  }
  
  public double[] receiveServerPose() {
    if(mConnected) {
      return nativeReceiveServerPose();      
    }
    return null;
  }

  public void release() {
    nativeOnDestroy();
  }

  private static native void nativeOnCreate();
  private static native boolean nativeConnect(String ip, int port);
  private static native void nativeOnPause();
  private static native void nativeOnDestroy();
  private static native void nativeFillImageBuffer(long matPtr);
  private static native boolean nativeTransmitImageBuffer();
  private static native double[] nativeReceiveServerPose();

}
