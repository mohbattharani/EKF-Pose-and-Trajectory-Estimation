/*
 
States: X = (x,y,z,x˙,y˙,z˙,x¨,y¨,z¨,ψ,θ,ϕ,ψ˙,θ˙,ϕ˙,ψ¨,θ¨,ϕ¨)


 * 
 */

#ifndef VISION_KALMAN_H
#define VISION_KALMAN_H

#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;
typedef Eigen::MatrixXd Matrix;

class vision_kalman
{
public:
vision_kalman ();
~vision_kalman();
void initKalmanFilter(int nStates, int nMeasurements, float covProcess, float covMeasurement); 
void prediction ();
void correction (Mat Rcam, Mat Tcam, float dt);
Matrix updateMeasurement (Mat Rcam, Mat Tcam);
void updateTransitionMatrix(float dt);
void eu2MAT ();
cv::Mat rot2euler(const cv::Mat & rotationMatrix);

Matrix X;

private:
  Matrix A, C;         // Wall distance estimation and measurement model
  Matrix Q, R;         // Model and measurement uncertainity
  Matrix P;         // Estimation variables 
  Matrix Z;            // measurements
  Matrix I;
};

#endif // VISION_KALMAN_H
