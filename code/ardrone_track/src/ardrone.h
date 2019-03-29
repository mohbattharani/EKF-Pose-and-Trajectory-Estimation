/*
 * ALL EKF related function and variables are defined are. 
 * As callbackIMU is called when imudata is recieved in main
 * SfMKLT is called when Image Frame is recieved
 */

#ifndef ARDRONE_H
#define ARDRONE_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/legacy/legacy.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxcore.h>

#include <math.h>
#include <tf/tf.h>

#include "vision_kalman.h"

using namespace ros;
using namespace std;
using namespace cv;
typedef Eigen::MatrixXd Matrix;
  
class ardrone
{
 #define totalStates 18
 #define alpha 0.8  
struct States{
    Matrix p;   		// position
    Matrix v;   		// velocity 
    Eigen::Quaterniond q;       // orientation quaternion
    //tf::Quaternion q;
    Matrix bg;  		//gyroscope bais
    Matrix ba;  		// accelerometer bias 
    Matrix g;   		//gravity
  };
  
public:
  ardrone();
  ~ardrone();
  
  void callbackIMU (const sensor_msgs::ImuConstPtr);
  void SfMKLT (const sensor_msgs::ImageConstPtr &msg);
  void initialize();
  vision_kalman vKF; 
  
  
  Matrix q, p, v, w, bg, ba, pf; 
  States Xest, Xt;   // state vector
  Matrix X;
  bool simulation;
    
private:
  float gaussian(float, float, float); 
  void prediction ();
  void correction ();
  void removeOutliers();
  double plotPoints ();
  void recoverCamPose ();
  void calResidual ();
  void computeH();
  void updateStateVision();
  void removeRow(Matrix& matrix, unsigned int rowToRemove);
  void removeColumn(Matrix& matrix, unsigned int colToRemove);
  void resizeP (Matrix& matrix, unsigned int idx);
  double distance (double, double, double, double);
  
  double dT;
  Matrix I3, O3, wx, ax;
  Matrix q0, q1;       // orientation in eular
  Matrix M;            // Rotation matrix for eular angle intergration
  Matrix M_a;          //accelerometer b to i  
  Matrix Xerr;         // error state vector
  Matrix n;            //System noise
  Matrix omega;
  Matrix gRi, iRc;          //Rotation matrix 
  Matrix am, wm;       //measured accleration and angular velocity
  
  Matrix Fx;           // Error transition matrix
  Matrix Fi;           //Input noise matrix
  Matrix Q;              // Model and measurement uncertainity
  Matrix P, Pimu;           // Estimation variables
  Matrix Vi, Ti, Ai, Gi; // Covariance  
  Matrix K, Z, R, Si;
  Matrix H, Hc, Ht, Hp, Hf;
  Matrix Fs, Gs, Fc, Gc, PHI, Qdk;
  Matrix r;                //residual  
  
  int N;
  double previousTimeIMU, previousTimeVis;
  bool needInit_P;
  bool initReq;
  bool resized;

  
  FeatureDetector* detector;
  DescriptorExtractor* extractor;
  FlannBasedMatcher matcher;
  vector<DMatch>matches;
  vector<cv::KeyPoint> keypointsNew, keypointsOld;
  Mat descriptorsOld, descriptorsNew;
  vector<Point2f> pointsNew, pointsOld;
  
  Mat cam_matrix;
  Mat dist_coeff;
  bool needtoinit;
  vector<uchar> status;
  vector<float> err;
  
  Mat rgbFrame, grayFrameNew, grayFrameOld;
  Mat opticalFlow;
  Mat Rc_rec, Tc_rec;
  Mat gP3, iP3;  //3D feature postion in global and IMU frames
  float ratio_;
};

#endif // ARDRONE_H
