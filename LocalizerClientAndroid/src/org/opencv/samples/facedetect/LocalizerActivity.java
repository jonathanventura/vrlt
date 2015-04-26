package org.opencv.samples.facedetect;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.WindowManager;

public class LocalizerActivity extends Activity implements CvCameraViewListener2 {

  private static final String TAG = "OCVSample::Activity";

  private Mat mRgba;
  private Mat mGray;
  private NativeCommunicationWraper mNativeCommunication;
  private boolean mServerResponse;

  private CameraBridgeViewBase mOpenCvCameraView;

  private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
    @Override
    public void onManagerConnected(int status) {

      if (status == LoaderCallbackInterface.SUCCESS) {
        Log.i(TAG, "OpenCV loaded successfully");
        System.loadLibrary("lacalizationClient");
        mOpenCvCameraView.enableView();
        //blocking until timeout or connection is established TODO make that better
        if (mNativeCommunication == null) {
          mNativeCommunication = new NativeCommunicationWraper();
        }
        mNativeCommunication.start();
      } else {
      }
      super.onManagerConnected(status);
    }
  };

  /** Called when the activity is first created. */
  @Override
  public void onCreate(Bundle savedInstanceState) {
    Log.i(TAG, "called onCreate");
    super.onCreate(savedInstanceState);
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

    setContentView(R.layout.localizer_activity_layout);

    mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.camera_view);
    mOpenCvCameraView.enableFpsMeter();
    mOpenCvCameraView.setCameraIndex(0);
    //mOpenCvCameraView.setMaxFrameSize(maxWidth, maxHeight);
    mOpenCvCameraView.setCvCameraViewListener(this);
  }

  @Override
  public void onPause() {
    super.onPause();
    if (mOpenCvCameraView != null)
      mOpenCvCameraView.disableView();
    mNativeCommunication.stop();
  }

  @Override
  public void onResume() {
    super.onResume();
    mServerResponse = true;
    OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
  }

  public void onDestroy() {
    super.onDestroy();
    mNativeCommunication.release();
    mOpenCvCameraView.disableView();
  }

  public void onCameraViewStarted(int width, int height) {
    mGray = new Mat();
    mRgba = new Mat();
  }

  public void onCameraViewStopped() {
    mGray.release();
    mRgba.release();
  }

  public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

    mRgba = inputFrame.rgba();
    mGray = inputFrame.gray();

    //TODO flip correctly
    //Core.flip(mRgba.t(), mRgba, 1);
    //Core.flip(mRgba.t(), mGray, 1);
    if (mNativeCommunication != null && mServerResponse){   
      // wait until the image in the native part is generated
      mNativeCommunication.fillImageBuffer(mGray);
      mServerResponse = false;
      //now send the data asynchronous and wait for a response
      new Thread(new Runnable() {        
        @Override
        public void run() {
          boolean imageSended = mNativeCommunication.transmitImageBuffer();
          
          System.out.println("imageSended " + imageSended);
          
          double[] pose = mNativeCommunication.receiveServerPose();
          
          if(pose != null)
            for (int i = 0; i < pose.length; i++) {
              System.out.print(pose[i] + " ");
            }
          System.out.println();
          
          mServerResponse = true;
        }
      }).start(); 
    }

    return mRgba;
  }

}
