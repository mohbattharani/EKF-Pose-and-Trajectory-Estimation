/*
 * 
 */

#include "vision_kalman.h"

vision_kalman::vision_kalman(){}
vision_kalman::~vision_kalman() {}

void vision_kalman::initKalmanFilter(int nStates, int nMeasurements, float covProcess, float covMeasurement)
{
  A = Matrix (nStates, nStates);
  X = Matrix (nStates, 1);
  P = Matrix (nStates, nStates);
  Q = Matrix (nStates, nStates);
  R = Matrix (nMeasurements, nMeasurements);
  C = Matrix (nMeasurements, nStates);
  Z = Matrix (nMeasurements, 1);
  I = Matrix (nStates, nStates).setIdentity();
  
  P.setIdentity();
  
  C(0,0) = 1;  // x
  C(1,1) = 1;  // y
  C(2,2) = 1;  // z
  C(3,9) = 1;  // roll
  C(4,10) = 1; // pitch
  C(5,11) = 1; // yaw
  
  Q.setIdentity()*(1e-5);
  R.setIdentity()*(1e-4);
  
}

void vision_kalman::prediction()
{
  X = A *X;
  P = A * P * A.transpose() + Q;
}

void vision_kalman::correction(Mat Rcam, Mat Tcam, float dt)
{
  Z = updateMeasurement(Rcam, Tcam);
  updateTransitionMatrix(dt);
  Matrix K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  X = X + K * (Z - C * X);
  P = (I - K * C) * P;
}

Matrix vision_kalman::updateMeasurement(Mat Rcam, Mat Tcam)
{
  Mat eulerMeasured = rot2euler (Rcam);
  Matrix eulerPos(6,1);
    eulerPos(0,0) = Tcam.at<double>(0);
    eulerPos(1,0) = Tcam.at<double>(1);
    eulerPos(2,0) = Tcam.at<double>(2);
    eulerPos(3,0) = eulerMeasured.at<double>(0);
    eulerPos(4,0) = eulerMeasured.at<double>(1);
    eulerPos(5,0) = eulerMeasured.at<double>(2);
  return eulerPos;  
}

void vision_kalman::updateTransitionMatrix(float dt)
{
    // position
  A(0,3) = dt;
  A(1,4) = dt;
  A(2,5) = dt;
  A(3,6) = dt;
  A(4,7) = dt;
  A(5,8) = dt;
  A(0,6) = 0.5*pow(dt,2);
  A(1,7) = 0.5*pow(dt,2);
  A(2,8) = 0.5*pow(dt,2);
  // orientation
  A(9,12) = dt;
  A(10,13) = dt;
  A(11,14) = dt;
  A(12,15) = dt;
  A(13,16) = dt;
  A(14,17) = dt;
  A(9,15) = 0.5*pow(dt,2);
  A(10,16) = 0.5*pow(dt,2);
  A(11,17) = 0.5*pow(dt,2); 
  
}


Mat vision_kalman::rot2euler(const Mat& rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else
  {
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }

  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;

  return euler;
}


void vision_kalman::eu2MAT()
{

}
