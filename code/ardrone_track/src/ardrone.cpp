/*
 * 
 * 
 */

#include "ardrone.h"

ardrone::ardrone (){
  // =========== allocate memory ================
  vKF.initKalmanFilter(18, 6, 1e-5, 1e-4);
  O3 = Matrix (3,3).setZero();
  I3 = Matrix (3,3).setIdentity();
  omega = Matrix (4,4).setZero();
  w = Matrix (3,1);
  q = Matrix (4,1);
  p = Matrix (3,1);
  v = Matrix (3,1);
  bg = Matrix (3,1).setRandom();
  ba = Matrix (3,1).setRandom();
  am = Matrix (3,1);
  wm = Matrix (3,1);
  wx = Matrix (3,3);
  ax = Matrix (3,3);
  n = Matrix (15,1);
  q0 = Matrix (3,1);
  q1 = Matrix (3,1);
  M = Matrix (3,3);
  M_a = Matrix (3,3);
  
  gRi = Matrix(3,3).setIdentity();   //Rotation from IMU to global Frame
  iRc = Matrix(3,3);   //Rotation from camera to IMU Frame
  Xerr = Matrix (totalStates,1);
  
  Xest.p = Matrix (3,1).setZero ();
  Xest.v = Matrix (3,1).setZero ();
  Xest.bg = Matrix (3,1).setZero ();
  Xest.ba = Matrix (3,1).setZero ();
  Xest.g = Matrix (3,1).setZero ();
  Xt.p = Matrix (3,1).setZero();
  
  Vi = Matrix (3,3);
  Ti = Matrix (3,3);
  Ai = Matrix (3,3).setIdentity();
  Gi = Matrix (3,3).setIdentity();
  Hc = Matrix (2,3);
  Ht = Matrix (3,3);
  Hp = Matrix (3,3);
  Hf = Matrix (3,3);
  
  Fs = Matrix (15, 15);
  Fx = Matrix (totalStates, totalStates).setZero();
  Fi = Matrix (totalStates, 12).setZero ();
  X = Matrix (totalStates, 1).setZero();  //State
  Pimu = Matrix (totalStates, totalStates).setZero();
  P = Matrix (totalStates, totalStates).setZero();
  Q = Matrix (12, 12).setZero();
  
  detector= new SiftFeatureDetector(
    300, // nFeatures
    4, // nOctaveLayers
    0.1, // contrastThreshold
    80, //edgeThreshold
    0.5 //sigma
  );
  
  extractor = new SiftDescriptorExtractor();
  opticalFlow = Mat::zeros(256, 256, CV_32F);
  simulation = false;
  cout<< "X:" << X <<endl;
  cout<< "P:" << P <<endl;
  cout<< "Q:" << Q <<endl;
}

// this destructor frees image viewing windows

ardrone::~ardrone()
{
  cv::destroyWindow("view");
  cv::destroyWindow("opticalFlow");
}

// ===================  initialize ===========================
// all variables are initialized here
void ardrone::initialize()
{
  previousTimeVis = Time::now().toSec();      //read time
  previousTimeIMU = ros::Time::now().toSec();
  iRc<< 0, -1, 0, 0, 0, -1, 1, 0, 1;   //rotation camera to IMU
  initReq = true;                       
  Xest.g << 0, 0, -9.8;               // gravity
  Xest.q.w() = 1;                    //initialized quoternion      
  X(3,0) = 1;                        //
  Fi <<  O3, O3, O3, O3,            
  I3, O3, O3, O3,
  O3, I3, O3, O3,
  O3, O3, I3, O3,
  O3, O3, O3, I3,
  O3, O3, O3, O3;
  
  // Covariance 
  Vi << 0.0004, 0.0002, 0.0025,
  0.0002, 0.0009, 0.0050,
  0.0025, 0.0050, 0.0771;
  Ti << 0.0679e-5, -0.0010e-5, 0.0014e-5,
  -0.0010e-5,  0.0503e-5,-0.0015e-5,
  0.0014e-5, -0.0015e-5, 0.1580e-5;
  
  Q << Vi, O3, O3, O3,
  O3, Ti, O3, O3, 
  O3, O3, Ai, O3,
  O3, O3, O3, Gi; 
  if (simulation){  //V_REP simulation camera parameters
    cam_matrix = (Mat_<double>(3, 3) <<
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);
    dist_coeff = (Mat_<double>(1, 5) <<0, 0, 0, 0, 0);
  }
  else {
    // ardrone_001 camera parameters
    cam_matrix = (Mat_<double>(3, 3) <<
    576.456375, 0, 335.080619,
    0, 578.273149, 165.737041,
    0, 0, 1);
    dist_coeff = (Mat_<double>(1, 5) <<-0.554867, 0.356153, 0.000984, 0.003222, 0.000000);
  }
  
  initReq = false;
  needtoinit = true;
  ratio_ = 0.8;
  N = 0;
  cv::namedWindow("view");
  cv::namedWindow("opticalFlow");
  cv::startWindowThread();   
  
}
// correction phase of Kalman filter
void ardrone::correction()
{
  calResidual();                  // Calculate residual
  computeH();                     // compute Jacobian
  R = Matrix (H.rows(), H.rows()).setIdentity() *(1e-5);
  cout<<"Size of R:"<<R.rows() <<" "<<R.cols()<<endl;
  Si = H * P * H.transpose() + R;
  cout <<"Gain:"<<Si.rows()<<" "<<Si.cols()<<endl;
  Matrix K = P * H.transpose() * Si.inverse();  //Kalman Gain
  cout <<"Kalman Gain:"<<K.rows()<<" "<<K.cols()<<endl;
  X = X + K * r;
  cout<<"State Vector:"<<X.rows()<<endl;
  P = P - K * Si *K.transpose();
}
// prediction
void ardrone::prediction()
{
  cout <<"prediction start... "<<endl;
  // Update states in actual state vector
  Matrix Pt(3,1);
  Pt = Xest.q.vec();
  X(0,0) = Pt(0,0);
  X(1,0) = Pt(1,0);
  X(2,0) = Pt(2,0);
  Pt = Xest.v;
  X(6,0) = Pt(0,0);
  X(7,0) = Pt(1,0);
  X(8,0) = Pt(2,0);
  Pt = Xest.p;
  X(12,0) = Pt(0,0);
  X(13,0) = Pt(1,0);
  X(14,0) = Pt(2,0);
  
  //Calculate wx and ax - the sckew symmetric matrix
  wx << 0,                    -(wm(2,0)-Xest.bg(2,0)), wm(1,0)-Xest.bg(1,0),
  wm(2,0)-Xest.bg(2,0),   0,                   -(wm(0,0)-Xest.bg(0,0)),
  -(wm(1,0)-Xest.bg(1,0)), wm(0,0)-Xest.bg(0,0),  0;
  
  ax << 0,                   -(am(2,0)-Xest.ba(2,0)), am(1,0)-Xest.ba(1,0),
  am(2,0)-Xest.ba(2,0),  0,                   -(am(0,0)-Xest.ba(0,0)),
  -(am(1,0)-Xest.ba(1,0)), am(0,0)-Xest.ba(0,0),  0;
  // Fx rerranged according to states
  Fx << gRi.transpose()*(wx)*dT, -I3*dT, O3,    O3,     O3, O3,
  O3,                       I3,    O3,    O3,     O3, O3,
  -gRi*(ax)*dT,              O3,    I3,   -gRi*dT, O3, I3*dT,
  O3,                       O3,    O3,    I3,     O3, O3,
  O3,                       O3,    I3*dT, O3,     I3, O3, 
  O3,                       O3,    O3,    O3,     O3, I3;
  
  /*   I3, I3*dT, O3,        		    O3,     O3,    O3,     
   *        O3, I3,   -gRi*(ax)*dT, 	   -gRi*dT, O3,    I3*dT,
   *        O3, O3,    gRi.transpose()*(wx)*dT, O3,     -I3*dT, O3,
   *	O3, O3,    O3,           	    I3,     O3,    O3,
   *	O3, O3,    O3,           	    O3,     I3,     O3,
   *	O3, O3,    O3,           	    O3,     O3,     I3;*/
  
  Fi <<  O3, O3, O3, O3,
  I3, O3, O3, O3,
  O3, I3, O3, O3,
  O3, O3, I3, O3,
  O3, O3, O3, I3,
  O3, O3, O3, O3;
  // ntegrating the covariances of an, wn, aw and ww  -> eq(246-248)	
  // rearranged as wn, ww, an, aw	 
  Q << Ti*dT*dT, O3,    O3,       O3,
  O3,       Gi*dT, O3,       O3, 
  O3,       O3,    Vi*dT*dT, O3,
  O3,       O3,    O3,       Ai*dT;
  
  Pimu = Fx * Pimu * Fx.transpose() + Fi* Q * Fi.transpose();   //432
  P.block (0,0,14,14) = Pimu.block(0,0,14,14);   //Move imu Covariance into actual Covariance Matrix
}

// Intergrate IMU values to predict new state
void ardrone::callbackIMU(const sensor_msgs::ImuConstPtr imuData)
{
  cout <<"State estimation running ..."<<endl;
  double newTime = ros::Time::now().toSec();
  dT = newTime - previousTimeIMU;
  //tf::Quaternion q2;  
  if (initReq){
    Xest.q.x() = imuData.get()->orientation.x;
    Xest.q.y() = imuData.get()->orientation.y;
    Xest.q.z() = imuData.get()->orientation.z;
    Xest.q.w() = imuData.get()->orientation.w;  
    initReq = false;
    previousTimeIMU = newTime;
    return; 
  }
  tf::Quaternion q2 = tf::createQuaternionFromRPY(X(0,0),X(1,0), X(2,0));
  if (abs (X(0,0))<1e5 and abs(X(1,0)) < 1e5){   //
    Xest.q.x() = q2.x();
    Xest.q.y() = q2.y();
    Xest.q.z() = q2.z();
    Xest.q.w() = q2.w();
    Xest.v(0,0) = X(6,0);
    Xest.v(1,0) = X(7,0);
    Xest.v(2,0) = X(8,0);
    Xest.p(0,0) = X(12,0);
    Xest.p(1,0) = X(13,0);
    Xest.p(2,0) = X(14,0);
  }
  gRi = (Xest.q.toRotationMatrix()).transpose();
  // cout <<"read meansurement"<<endl;
  // read meansurement
  am<< imuData.get()->linear_acceleration.x,imuData.get()->linear_acceleration.y,imuData.get()->linear_acceleration.z;
  wm<< imuData.get()->angular_velocity.x, imuData.get()->angular_velocity.y, imuData.get()->angular_velocity.z;
  // estimate noise
  
  Xest.p += Xest.v * dT/100; //+ 0.5 *(gRi *am + Xest.g)*dT*dT;         // update position   
  Xest.v += (gRi * am + Xest.g) * dT;   // convert acceleration to intertial frame and integrate to compute velocity  
  
  // estimate states  (Nominal states)   
  // ==== Add ab and wb  in the equations**.
  //q0 << Xest.q.vec();	 
  double roll = q0(0,0), pitch = q0(1,0), yaw = q0(2,0);
  omega <<0,        wm(2,0), -wm(1,0), wm(0,0), 
  -wm(2,0),  0,        wm(0,0), wm(1,0),
  wm(1,0), -wm(0,0),  0,       wm(2,0),
  -wm(0,0), -wm(1,0), -wm(2,0), 0;	  
  // Intergrate gyroscope;
  M << 1, sin(roll)*tan(pitch), cos(roll)*tan(roll),
  0, cos(roll)          , -sin(roll),
  0, sin(roll)/cos(pitch), cos(roll)/cos(pitch);
  
  q1 = Xest.q.vec() + gRi.transpose()*(wm)*dT;    
  
  // body frame to intertial frame transformation Matrix
  M_a << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw), sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch),
  cos(pitch)*sin(yaw), cos(roll)*cos(yaw)+sin(roll)*sin(yaw)*sin(pitch), cos(roll)*sin(yaw)*sin(pitch)-cos(yaw)*cos(roll),
  -sin(pitch), 	cos(pitch)*sin(roll), 	cos(roll)*cos(pitch);
  
  q2 = tf::createQuaternionFromRPY(q1(0,0),q1(1,0), q1(2,0));
  Xest.q.x() = alpha * imuData.get()->orientation.x + (1-alpha)* q2.x();
  Xest.q.y() = alpha * imuData.get()->orientation.y + (1-alpha)* q2.y();
  Xest.q.z() = alpha * imuData.get()->orientation.z + (1-alpha)* q2.z();
  Xest.q.w() = alpha * imuData.get()->orientation.w + (1-alpha)* q2.w();
  gRi =  (Xest.q.toRotationMatrix()).transpose();  
  
  prediction();
  previousTimeIMU = newTime;
  
}


// ***************************** VISION *********************************************

void ardrone::SfMKLT(const sensor_msgs::ImageConstPtr& msg)
{
  TermCriteria termcrit (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
  Size subPixWinSize (10,10), winSize(15,15);
  int i, k;
  double newTime = ros::Time::now().toSec();
  dT = newTime - previousTimeVis;
  
  vKF.prediction();  // predict camera orientation using linear Kalman Filter
  
  cv_bridge::CvImagePtr cv_ptr;
  
  // Bridge ros_image to opencv image
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8" );
    rgbFrame = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // convert from brg to grayscale
  cv::cvtColor(rgbFrame,grayFrameNew,CV_BGR2GRAY);
  
  // image processing task    
  if (needtoinit){  // need to init by calculating features
    detector->detect ( grayFrameNew, keypointsNew);     //detect SIFT Features
    for (int j =0; j<keypointsNew.size();j++){          // Convert features to points  
      pointsNew.push_back(keypointsNew[j].pt);
   }
    needInit_P = true; 
    cout<<"initialization completed... "<<endl;
    needtoinit = false;
  }
  else {    
    calcOpticalFlowPyrLK(grayFrameOld, grayFrameNew, pointsOld ,pointsNew, status,err, winSize, 3, termcrit,0,0.001 );
    cout <<"status: ";
    
    if (plotPoints()<20)  // If there is no motion then skip following steps 
      goto swaping;
    
    removeOutliers();    // remove unmatched features and also update P and X sizes           
    
    if (needInit_P){                  //P is initialized when we actually initialize features matrix 
      N = pointsNew.size();
      P = Matrix (3*N+15, 3*N+15).setIdentity();
      X.conservativeResize(3*N+15, 1);    //resize state vector
      needInit_P = false;
      cout <<"P initialization ... "<<endl;
    }
    if (resized){
      cout<<"After removing outliers RANSAC:"<<pointsNew.size()<<" "<<pointsOld.size()<<endl;
      cout<<"After removing outliers P size:"<<P.rows()<<","<<P.cols()<<endl;
    }
    cout <<"After erasing outliers: "<<pointsNew.size()<<endl;
    if (pointsNew.size() <10 or pointsNew.size()>500){
      needtoinit =true;
    }
    else if(pointsOld.size()>3 and pointsNew.size()>3) {   //If matched features more than 3 then recover camera pose
      recoverCamPose();
      vKF.correction(Rc_rec, Tc_rec, dT);            // Correct camera pose predicted earlier using Linear KF
      updateStateVision();                           // 
      correction();                                 // correction phase of EKF
    }	
  }
  
  swaping:
  swap(pointsOld, pointsNew);
  swap(keypointsOld, keypointsNew);
  grayFrameNew.copyTo(grayFrameOld); 
  //cout <<"State Estimated KF:" << vKF.X <<endl;
  cout<<"Points after swaping: "<<pointsNew.size()<<" "<<pointsOld.size()<<endl;
  cout<<"sawping completed.."<<endl;
  previousTimeVis = newTime; 
  // visualize  
  try
  {
    imshow("view", rgbFrame);
    imshow("opticalFlow", opticalFlow);
    //	cout<<points1<<endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s'.", e.what());
    return;
  }     
}


void ardrone::recoverCamPose()
{
  // **** recover 3D Points ***** 
  
  Mat fundamental = findFundamentalMat( pointsOld, pointsNew, CV_FM_RANSAC, 1.0, 0.99 );
  Mat essential = cam_matrix.t() * fundamental * cam_matrix; 
  
  /* Find the projection matrix between those two images */
  SVD svd( essential );
  static const Mat W = (Mat_<double>(3, 3) <<
  0, -1, 0,
  1, 0, 0,
  0, 0, 1);
  
  static const Mat W_inv = W.inv();
  
  Mat_<double> R1 = svd.u * W * svd.vt;
  Mat_<double> T1 = svd.u.col( 2 );
  
  Mat_<double> R2 = svd.u * W_inv * svd.vt;
  Mat_<double> T2 = -svd.u.col( 2 );
  
  static const Mat P1 = Mat::eye(3, 4, CV_64FC1 );
  Mat P2 =( Mat_<double>(3, 4) <<
  R1(0, 0), R1(0, 1), R1(0, 2), T1(0),
	    R1(1, 0), R1(1, 1), R1(1, 2), T1(1),
	    R1(2, 0), R1(2, 1), R1(2, 2), T1(2));
  
  /*  Triangulate the points to find the 3D homogenous points in the world space
   *        Note that each column of the 'out' matrix corresponds to the 3d homogenous point
   */
  gP3.empty();   
  triangulatePoints( P1, P2, pointsOld, pointsNew, gP3 );
  /* Since it's homogenous (x, y, z, w) coord, divide by w to get (x, y, z, 1) */    
  vector<Mat> splitted;
  splitted.push_back(gP3.row(0) / gP3.row(3)); 
  splitted.push_back(gP3.row(1) / gP3.row(3));  
  splitted.push_back(gP3.row(2) / gP3.row(3));  
  gP3.empty();
  merge( splitted, gP3 );
  
  // estimate camera motion 
  // Mat rvec, tvec; 
  solvePnP(gP3, pointsNew, cam_matrix, dist_coeff, Rc_rec, Tc_rec);    
  cout<<"Rrec:" << Rc_rec<<endl;
  cout<<"Trec: "<<Tc_rec<<endl;
  
  //cout <<"3D points: "<< gP3.size() <<endl<<endl;
  //cout <<"point 3D:" << gP3.col(0);
}

void ardrone::calResidual()
{
  vector<cv::Point2f> pointsProj, pointsTrue;
  pointsTrue = pointsNew;
  int i, k;
  
  Matrix pointTemp(3,1), point0;
  cout<<"Errors: ";
  projectPoints(gP3, Rc_rec, Tc_rec, cam_matrix, dist_coeff, pointsProj);
  cout<< "size of 3D points: "<<gP3.size() <<endl;
  cout <<"size of recovered:"<<pointsProj.size()<<endl; 
  r = Matrix (2*pointsTrue.size(), 1);   //resize 'r' matrix double to total number of features.
  
  k=0;
  for (i=0; i<pointsProj.size(); i++){
    r(k,0) = (pointsTrue[i].x - pointsProj[i].x);     // Calculate residual and update in "r" matrix
    r(k+1,0) = (pointsTrue[i].y - pointsProj[i].y);
    k++;
  }
  cout <<endl;
  
}

// compute Camera meansurement Jacobian
void ardrone::computeH()
{
  N = pointsNew.size();
  H = Matrix(2*N, 3*N+15); 
  Matrix H1 (3,3*N+15);
  double px, py, pz;    //location of feature 'f' in IMU frame;
  int i =0;
  
  for (i=0; i<N; i++){
    px = gP3.col(i).at<double>(0);
    py = gP3.col(i).at<double>(1);
    pz = gP3.col(i).at<double>(2);
    
    Hc << 1/pz, 0 , -px/py, 0, 1/pz, -py/pz;
    Ht << 0, -pz, py, p(2,0), 0, -pz, -py, px, 0;
    Hp << -gRi.transpose();
    Hf << gRi.transpose();
    H1.block(0,0, 3, 3) = Ht;
    H1.block(0,11, 3, 3) = -gRi.transpose();
    H1.block(0,16+i, 3, 3) = gRi.transpose(); 
    H.block(i, 0, 2, 3*N+15) = Hc * H1;
  }
  cout <<" Size of H:" <<H.rows()<<","<<H.cols();
  cout <<" Size of P:" <<P.rows()<<","<<P.cols()<<endl;
}

// visualize features and motion on windows
double ardrone::plotPoints()
{
  int i, k;
  double dist =0;
  cout<<"Start ploting ";
  for (i=k=0; i<pointsOld.size(); i++){
    //cout<<"Starting drawing arrow..."<<endl;
    //  cout<<points2<<endl;
    if (pointsNew[i].x - pointsOld[i].y > 0){
      line(rgbFrame,pointsNew[i], pointsOld[i], Scalar(0,0,255),1, 1, 0); 
      circle(rgbFrame,pointsNew[i], 2, Scalar(255,0,0),1, 1, 0); 
      line(opticalFlow,pointsNew[i], pointsOld[i], Scalar(0,0,255),1, 1, 0); 
      circle(opticalFlow,pointsNew[i], 1, Scalar(255,0,0),1, 1, 0); 
    } 
    else {
      line(rgbFrame,pointsNew[i], pointsOld[i], Scalar(0,0,255),1, 1, 0); 
      circle(rgbFrame,pointsNew[i], 2, Scalar(255,0,0),1, 1, 0); 
      line(opticalFlow,pointsNew[i], pointsOld[i], Scalar(0,0,255),1, 1, 0); 
      circle(opticalFlow,pointsNew[i], 1, Scalar(255,0,0),1, 1, 0); 
    }
    
    dist += distance(pointsNew[i].x, pointsOld[i].y, pointsOld[i].x, pointsOld[i].y);
  }
  
  cout <<"Total distance covered by features: "<<dist <<endl;
  cout <<" done"<<endl;
  return dist;
}

// remove unmatched and out of window features and update P & X
void ardrone::removeOutliers(){
  
  float reprojectionThreshold =500;
  Mat homography;
  int indexCorrection = 0;
  vector<cv::Point2f> srcPoints;
  vector<cv::Point2f> dstPoints; 
  // Find homography matrix and get inliers mask
  vector<unsigned char> inliersMask(keypointsNew.size());
  
  homography = cv::findHomography(pointsNew, pointsOld,CV_FM_RANSAC, reprojectionThreshold, inliersMask);
  cout<<"Before removing outliers RANSAC:"<<pointsNew.size()<<" "<<pointsOld.size()<<endl;
  cout<<"Before removing outliers P size:"<<P.rows()<<","<<P.cols()<<endl;
  resized = false;
  
  for(int i=0; i<status.size(); i++){  
    Point2f pt = pointsOld.at(i- indexCorrection);
    if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0) or inliersMask[i]==0)	{
      if((pt.x<0)||(pt.y<0))	{
	status.at(i) = 0;
      }
      resized = true;
      
      pointsNew.erase (pointsNew.begin() + (i - indexCorrection));
      pointsOld.erase (pointsOld.begin() + (i - indexCorrection));
      // cout<<" "<<P.rows()<<endl;
      if (!needInit_P){
	resizeP (P, i-indexCorrection);
      }
      indexCorrection++;
    }
  } 
}

// Pose estimated using Linear KF from recovered camera pose - NOT USED ANY WHERE 
void ardrone::updateStateVision()
{
  tf::Quaternion q2 = tf::createQuaternionFromRPY(vKF.X(0,0),vKF.X(1,0), vKF.X(2,0));
  Xt.p(0,0) = vKF.X(0,0);
  Xt.p(1,0) = vKF.X(1,0);
  Xt.p(2,0) = vKF.X(2,0);
  Xt.q.x() = q2.x();
  Xt.q.y() = q2.y();
  Xt.q.z() = q2.z();
  Xt.q.w() = q2.w();
}

// resize Covariance "P" and state "X" 
void ardrone::resizeP(Matrix& matrix, unsigned int idx)
{
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols();
  unsigned int idxStart = idx*3;
  unsigned int idxes = numRows-idxStart-3;
  
  if (idxes <numRows and idxes <numCols){
    matrix.block (idxStart,0, idxes,numCols)  = matrix.block(idxStart+3, 0, idxes, numCols);  //remove 3 rows
    matrix.block (0, idxStart, numRows,idxes) = matrix.block(0, idxStart+3, numRows, idxes);  //romove 3 colums
    X.block (idxStart,0, idxes,0) = X.block (idxStart+3,0,idxes,0); 
  }
  matrix.conservativeResize(numRows-3,numCols-3);
  X.conservativeResize(numRows-3, 1);
}


void ardrone::removeRow(Matrix& matrix, unsigned int rowToRemove)
{
  unsigned int numRows = matrix.rows()-1;
  unsigned int numCols = matrix.cols();
  // cout <<" "<<rowToRemove<<" "<< matrix.rows();
  if( rowToRemove < numRows )
    matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);
  matrix.conservativeResize(numRows,numCols);
  // cout <<" "<<matrix.rows();
}
void ardrone::removeColumn(Matrix& matrix, unsigned int colToRemove)
{
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols()-1;
  
  if( colToRemove < numCols )
    matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);
  
  matrix.conservativeResize(numRows,numCols);
}


// Calculate Gaussian Probablity
float ardrone::gaussian(float mu, float sigma, float x)
{
  float pi = 3.14;
  float val = exp((-pow((x-mu),2))/(2*pow(sigma,2))) /sqrt(2*pi* pow (sigma,2)) ;
  //std::cout<<val << "  "<<mu<<"  "<<sigma <<"  " <<x << endl;
  return  val;
}


double ardrone::distance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow (x2-x1,2) + pow (y2-y1,2) );
}
