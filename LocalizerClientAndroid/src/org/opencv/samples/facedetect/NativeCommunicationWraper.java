package org.opencv.samples.facedetect;

import org.opencv.core.Mat;

public class NativeCommunicationWraper {
  boolean mConnected;

  public NativeCommunicationWraper() {
    nativeOnCreate();
    mConnected = false;
  }

  public void start() {
    mConnected = nativeOnStart();
  }

  public void stop() {
    nativeOnPause();
  }

  public void fillImageBuffer(Mat imageGray) {
    nativeFillImageBuffer(imageGray.getNativeObjAddr());
  }

  public boolean transmitImageBuffer() {
    if(mConnected) {
      return nativeTransmitImageBuffer();      
    }
    return false;
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
  private static native boolean nativeOnStart();
  private static native void nativeOnPause();
  private static native void nativeOnDestroy();
  private static native void nativeFillImageBuffer(long matPtr);
  private static native boolean nativeTransmitImageBuffer();
  private static native double[] nativeReceiveServerPose();

}
