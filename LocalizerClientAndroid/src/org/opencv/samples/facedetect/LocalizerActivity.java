package org.opencv.samples.facedetect;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
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
        if (mNativeCommunication == null) {
          mNativeCommunication = new NativeCommunicationWraper();
        }
        connectToServer();
        mOpenCvCameraView.enableView();
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
    mOpenCvCameraView.setMaxFrameSize(800, 480); // use this resolution to avoid an incorrect aspect ratio set by the cam view TODO!!!!
    mOpenCvCameraView.setCvCameraViewListener(this);
    
    mServerResponse = true;
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
    OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
  }

  private void connectToServer() {
    if (mServerResponse) {
      mServerResponse = false;
      // establish server connection asynch
      new Thread(new Runnable() {
        @Override
        public void run() {
          mNativeCommunication.connect("10.249.153.197", 12345);
          mServerResponse = true;
        }
      }).start();
    }
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

    if (mNativeCommunication != null && mServerResponse) {
      // copy compressed image data for asynch transmit
      mNativeCommunication.fillImageBuffer(mGray);
      mServerResponse = false;

      // now send the data asynchronous and wait for a response
      new Thread(new Runnable() {
        @Override
        public void run() {
          double[] pose = null;

          // if the image is transmitted successfully to the server receive the pose from the server
          // blocking until timeout, if connection is not established transmitImageBuffer reconnects
          if (mNativeCommunication.transmitImageBuffer()) {
            // blocking until pose is received or a timeout occurred
            pose = mNativeCommunication.receiveServerPose();
          }

          logServerPose(pose);
          mServerResponse = true;

        }
      }).start();
    }
    return mRgba;
  }

  private void logServerPose(double[] pose) {
    if (pose != null)
      for (int i = 0; i < pose.length; i++) {
        System.out.print(pose[i] + " ");
      }
    System.out.println();
  }

}
