// Using SIFT feature detector 
/* Method to install nonfree library with ROS
  
 sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
 sudo apt-get update
 sudo apt-get install libopencv-nonfree-dev

# Add folowing script in CMakeLists.text
  
target_link_libraries(ProjectName ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv_nonfree ) 

*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <geometry_msgs/Twist.h>
#include "opencv2/legacy/legacy.hpp"

//#include <opencv2/calib3d/calib3d.hpp> // for homography

using namespace std;
using namespace ros;
using namespace cv;

Mat frame, rgbFrame, grayFrames, previousGrayFrame;
Mat opticalFlow = Mat::zeros(256, 256, CV_32F);
vector<Point2f> pointsNew, pointsOld;
Point2f relativePos, diff;
vector<uchar> status;
vector<float> err;
double V = 0, previousTime;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void calVelocity ();
double euclidDistance(Mat& vec1, Mat& vec2);
int nearestNeighbor(Mat& vec, vector<KeyPoint>& keypoints, Mat& descriptors);
void findPairs(vector<KeyPoint>&, Mat&, vector<KeyPoint>&, Mat&, vector<Point2f>& , vector<Point2f>&);
void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);
void SFM();


int main(int argc, char **argv)
{
    ros::init(argc, argv, "quadrotor_sift");
    ros::NodeHandle nh;
    
    cv::namedWindow("view");
    cv::namedWindow("opticalFlow");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    
    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
    ros::Publisher cmdPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist cmd;
    
    ros::Rate rate(50);
    previousTime = ros::Time::now().toSec();
    relativePos.x = 0;
    relativePos.y = 0;
    
    cout <<"testing"<<endl;
    int x = 0;
    while (ros::ok())
    {
      // if ( x == 0) { SFM(); x = 1;
      // cout <<"*************completed **********"<<endl;}
      
      calVelocity ();
      cmd.linear.x = V;
      cmd.linear.y = V;
      cmd.linear.z = 0;
      cmd.angular.x = V*100;
      cmd.angular.y = V*100;
      cmd.angular.z = 0;
      //cmdPub.publish(cmd);
      
      ros::spinOnce();	
      rate.sleep();
    }
    cv::destroyWindow("view");
    cv::destroyWindow("opticalFlow");
    return 0;
}

//======================================================
bool needtoinit = true;
TermCriteria termcrit (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
Size subPixWinSize (10,10), winSize(31,31);
int i,k;
// *******  SIFT feature detector and feature extractor *************
 //  SiftFeatureDetector detector;
FeatureDetector* detector= new SiftFeatureDetector(
                                     30, // nFeatures
                                     4, // nOctaveLayers
                                     0.1, // contrastThreshold
                                     80, //edgeThreshold
                                     1.5 //sigma
);
DescriptorExtractor* extractor = new SiftDescriptorExtractor();
vector<cv::KeyPoint> keypointsNew, keypointsOld;
Mat descriptorsOld, descriptorsNew;
FlannBasedMatcher matcher;
vector<DMatch>matches;
	    
// ardrone_001
Mat cam_matrix = (Mat_<double>(3, 3) <<
                    576.456375, 0, 335.080619,
                    0, 578.273149, 165.737041,
                    0, 0, 1);
Mat dist_coeff = (Mat_<double>(1, 5) <<-0.554867, 0.356153, 0.000984, 0.003222, 0.000000);
//void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int minHessian = 1200;
SurfFeatureDetector detectorSURF( minHessian );
SurfDescriptorExtractor extractorSURF;

void SFM() {
    
     Mat img1 = imread( "/home/mohbat/Pictures/ardrone1image1.png" );
     Mat img2 = imread( "/home/mohbat/Pictures/ardrone1image2.png" );  
     cvtColor( img1, previousGrayFrame, CV_BGR2GRAY );
     cvtColor( img2, grayFrames, CV_BGR2GRAY );
     detector->detect ( previousGrayFrame, keypointsOld);
     extractor->compute (previousGrayFrame, keypointsOld, descriptorsOld );
     detector->detect ( grayFrames, keypointsNew);
     extractor->compute (grayFrames, keypointsNew, descriptorsNew );
     matcher.match(descriptorsOld, descriptorsNew, matches);
     cout<<"Matches: "  <<" "<<matches.size() <<endl<<endl;
     double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( i = 0; i < descriptorsNew.rows; i++ )
      { 
	double dist = matches[i].distance;
	if( dist < min_dist ) 
	  min_dist = dist;
	if( dist > max_dist ) 
	  max_dist = dist;
      }
      cout <<endl;
      cout<<"-- Max dist : "<< max_dist<<" ";
      cout<<"-- Min dist : "<< min_dist<<endl;
      vector< DMatch > good_matches;
  
  for(  i = 0; i < descriptorsNew.rows; i++ )
  { if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
    good_matches.push_back( matches[i]);     
   }
  }
  cout <<"After removing large distances: "<<good_matches.size()<<endl;
  pointsNew.clear();
  pointsOld.clear();
  for( i = 0; i < (int)matches.size(); i++ )
  { 
   // cout<<"-- Good Match :"<<i<< " Keypoint New: " <<good_matches[i].queryIdx<< "-- Keypoint old: "<< good_matches[i].trainIdx <<endl<<endl; 
    
    pointsNew.push_back(keypointsNew[matches[i].trainIdx].pt);
    pointsOld.push_back(keypointsOld[matches[i].queryIdx].pt);
    
  }
 // calcOpticalFlowPyrLK(previousGrayFrame, grayFrames, pointsOld ,pointsNew, status,err, winSize, 3, termcrit,0,0.001 );
     
  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches(previousGrayFrame, keypointsOld, grayFrames, keypointsNew, 
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  cout <<"show matched...";
  imshow( "view", img_matches );
  imwrite( "SIFT_Matching_1.jpg", img_matches );
  cout <<"done"<<endl;
 //   waitKey(0);
   /* 
     // **** recover 3D Points ***** 
      
     Mat fundamental = findFundamentalMat( pointsOld, pointsNew, RANSAC, 3.0, 0.99 );
     Mat essential = cam_matrix.t() * fundamental * cam_matrix; 
  
    // Find the projection matrix between those two images 
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
  
      //  Triangulate the points to find the 3D homogenous points in the world space
      //  Note that each column of the 'out' matrix corresponds to the 3d homogenous point
      
     Mat out;    
     triangulatePoints( P1, P2, pointsOld, pointsNew, out );
     // Since it's homogenous (x, y, z, w) coord, divide by w to get (x, y, z, 1) //   
     vector<Mat> splitted;
     splitted.push_back(out.row(0) / out.row(3)); 
     splitted.push_back(out.row(1) / out.row(3));  
     splitted.push_back(out.row(2) / out.row(3));  
     out.empty();
     merge( splitted, out );
     
      // estimate camera motion 
      Mat rvec, tvec; 
      solvePnP(out, pointsNew, cam_matrix, dist_coeff, rvec, tvec);    
      cout<<"rvec:" << rvec<<endl;
      cout<<"t: "<<tvec <<endl;
     
     //cout <<"3D points: "<< out.size() <<endl<<endl;
     //cout <<"point 3D:" << out.col(0); 
     
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
	line(rgbFrame,pointsNew[i], pointsOld[i], Scalar(0,255,0),1, 1, 0); 
        circle(rgbFrame,pointsNew[i], 2, Scalar(255,0,0),1, 1, 0); 
	line(opticalFlow,pointsNew[i], pointsOld[i], Scalar(0,255,0),1, 1, 0); 
        circle(opticalFlow,pointsNew[i], 1, Scalar(255,0,0),1, 1, 0); 
      }
    }
    
     detector->detect ( grayFrames, keypointsNew);
     extractor->compute (grayFrames, keypointsNew, descriptorsNew );
     for (int j =0; j<keypointsNew.size();j++){
     pointsNew.push_back(keypointsNew[j].pt);
    // cout<<"j: "<<j<< "  "<<(Keypoints1[j].pt) <<endl;
    }
    cout<<"Loop completed... "<<endl;
    }
   }
 
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
      }     */
} 


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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
    cv::cvtColor(rgbFrame,grayFrames,CV_BGR2GRAY);
    
   // image processing task    
   if (needtoinit){  // need to init by calculating features
     detector->detect ( grayFrames, keypointsNew);
     extractor->compute (grayFrames, keypointsNew, descriptorsNew );
    // cout<<Keypoints1.size()<<endl;
     
     for (int j =0; j<keypointsNew.size();j++){
       pointsNew.push_back(keypointsNew[j].pt);
//    // cout<<"j: "<<j<< "  "<<(Keypoints1[j].pt) <<endl;
     }
//     relativePos.x = pointsNew[1].x;
//     relativePos.y = pointsNew[1].y;
    cout<<"Loop completed... "<<endl;
    needtoinit = false;
   }
   else {
 /*    detector->detect ( grayFrames, keypointsNew);
     extractor->compute (grayFrames, keypointsNew, descriptorsNew );
     matcher.match(descriptorsNew, descriptorsOld, matches);
     cout<<"Matches: "  <<" "<<matches.size() <<endl<<endl;
    double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( i = 0; i < descriptorsNew.rows; i++ )
  { 
    double dist = matches[i].distance;
    if( dist < min_dist ) 
      min_dist = dist;
    if( dist > max_dist ) 
      max_dist = dist;
  }
  cout <<endl;
  cout<<"-- Max dist : "<< max_dist<<" ";
  cout<<"-- Min dist : "<< min_dist<<endl;
    std::vector< DMatch > good_matches;

  for(  i = 0; i < descriptorsNew.rows; i++ )
  { //if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
    good_matches.push_back( matches[i]);     
   //}
  }
  cout <<"After removing large distances: "<<good_matches.size()<<endl;
  pointsNew.clear();
  pointsOld.clear();
  for( i = 0; i < (int)good_matches.size(); i++ )
  { 
   // cout<<"-- Good Match :"<<i<< " Keypoint New: " <<good_matches[i].queryIdx<< "-- Keypoint old: "<< good_matches[i].trainIdx <<endl<<endl; 
    
    pointsNew.push_back(keypointsNew[good_matches[i].queryIdx].pt);
    pointsOld.push_back(keypointsOld[good_matches[i].trainIdx].pt);
    
  }
  */
  //-- Draw only "good" matches
  //Mat img_matches;
  //drawMatches( grayFrames, keypointsNew, previousGrayFrame, keypointsOld,
   //            good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
    //           vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  calcOpticalFlowPyrLK(previousGrayFrame, grayFrames, pointsOld ,pointsNew, status,err, winSize, 3, termcrit,0,0.01 );
      int indexCorrection = 0;
     for(  i=0; i<status.size(); i++)
	{  
	  Point2f pt = pointsOld.at(i- indexCorrection);
	  if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  pointsNew.erase (pointsNew.begin() + (i - indexCorrection));
     		  pointsOld.erase (pointsOld.begin() + (i - indexCorrection));
     		  indexCorrection++;
	  }
	}
	cout <<"After erasing outliers: "<<pointsNew.size()<<endl;
	if (pointsNew.size() <1 or pointsNew.size()>500){
	  needtoinit =true;
	  return;
	}
	
  if (pointsNew.size()< 10){
     detector->detect ( grayFrames, keypointsNew);
     extractor->compute (grayFrames, keypointsNew, descriptorsNew );
    // cout<<Keypoints1.size()<<endl;
     
     for (int j =0; j<keypointsNew.size();j++){
       pointsNew.push_back(keypointsNew[j].pt);
//    // cout<<"j: "<<j<< "  "<<(Keypoints1[j].pt) <<endl;
     }
    cout <<"feature computed"<<endl;
    return;
  }
  
  //-- Show detected matches
  //imshow( "Good Matches", img_matches );
  
  cout <<"Matched pointsNew: "<< pointsNew.size()<<endl;  
     //findPairs (keypointsOld, descriptorsOld, keypointsNew, descriptorsNew, pointsOld, pointsNew);
   // calcOpticalFlowPyrLK(previousGrayFrame, grayFrames, pointsOld ,pointsNew, status,err, winSize, 3, termcrit,0,0.001 );
     //featureTracking(grayFrames, previousGrayFrame, pointsNew,pointsOld, status); //track those features to image2
     

     //Mat img_matches;
    //drawMatches(grayFrames,keypointsNew, previousGrayFrame,keypointsOld,matches,img_matches);
    //imshow("view",img_matches);
    //waitKey(0);
     //cout<<"Number of old points:"<<pointsOld.size()<<" Number of new points" <<pointsNew.size()<<endl<<endl;
     // cout <<"pointsNew: " << pointsNew <<endl;
    // cout <<"pointsOld: " << pointsOld <<endl<<endl;
    if (pointsOld.size()>3 and pointsNew.size()>3) { 
    
     // **** recover 3D Points ***** 
      
     Mat fundamental = findFundamentalMat( pointsOld, pointsNew, RANSAC, 3.0, 0.99 );
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
        Note that each column of the 'out' matrix corresponds to the 3d homogenous point
      */
     Mat out;    
     triangulatePoints( P1, P2, pointsOld, pointsNew, out );
     /* Since it's homogenous (x, y, z, w) coord, divide by w to get (x, y, z, 1) */    
     vector<Mat> splitted;
     splitted.push_back(out.row(0) / out.row(3)); 
     splitted.push_back(out.row(1) / out.row(3));  
     splitted.push_back(out.row(2) / out.row(3));  
     out.empty();
     merge( splitted, out );
     
      // estimate camera motion 
      Mat rvec, tvec; 
      solvePnP(out, pointsNew, cam_matrix, dist_coeff, rvec, tvec);    
      cout<<"rvec:" << rvec<<endl;
      cout<<"t: "<<tvec <<endl;
     
     //cout <<"3D points: "<< out.size() <<endl<<endl;
     //cout <<"point 3D:" << out.col(0); 
     
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
	line(rgbFrame,pointsNew[i], pointsOld[i], Scalar(0,255,0),1, 1, 0); 
        circle(rgbFrame,pointsNew[i], 2, Scalar(255,0,0),1, 1, 0); 
	line(opticalFlow,pointsNew[i], pointsOld[i], Scalar(0,255,0),1, 1, 0); 
        circle(opticalFlow,pointsNew[i], 1, Scalar(255,0,0),1, 1, 0); 
      }
    }
    
     detector->detect ( grayFrames, keypointsNew);
     extractor->compute (grayFrames, keypointsNew, descriptorsNew );
     for (int j =0; j<keypointsNew.size();j++){
     pointsNew.push_back(keypointsNew[j].pt);
    // cout<<"j: "<<j<< "  "<<(Keypoints1[j].pt) <<endl;
    }
    cout<<"Loop completed... "<<endl;
    }
   }
 //  cout<<points1 <<endl;
   
   swap(pointsOld,pointsNew);
   swap(keypointsNew, keypointsOld);
   swap(descriptorsNew, descriptorsOld);
   
   pointsNew.clear();
   grayFrames.copyTo(previousGrayFrame); 
  // cout<<"sawping completed.."<<endl;
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

void calVelocity (){
  static double lastmeasureTime =0;
  double newTime = ros::Time::now().toSec();
  double dTime = newTime - previousTime;
  lastmeasureTime += dTime;    
 //  cout<<"Timer:"<<lastmeasureTime<<endl; 
    if (lastmeasureTime< 10)
      V = 0.5;
    else{
      V= -0.5;
      if ( lastmeasureTime>20)
	lastmeasureTime = 0;
    }
    previousTime = newTime;
}

/**
 * Calculate euclid distance
 * https://gist.github.com/thorikawa/3398619
 */
double euclidDistance(Mat& vec1, Mat& vec2) {
  double sum = 0.0;
  int dim = vec1.cols;
  for (int i = 0; i < dim; i++) {
    sum += (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i)) * (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i));
  }
  return sqrt(sum);
}

/**
 * Find the index of nearest neighbor point from keypoints.
 */
const double THRESHOLD = 400;
int nearestNeighbor(Mat& vec, vector<KeyPoint>& keypoints, Mat& descriptors) {
  int neighbor = -1;
  double minDist = 1e6;
  
  for (int i = 0; i < descriptors.rows; i++) {
    KeyPoint pt = keypoints[i];
    Mat v = descriptors.row(i);
    double d = euclidDistance(vec, v);
    //printf("%d %f\n", v.cols, d);
    if (d < minDist) {
      minDist = d;
      neighbor = i;
    }
  }
  
  if (minDist < THRESHOLD) {
    return neighbor;
  }
  
  return -1;
}

/**
 * Find pairs of points with the smallest distace between them
 */
void findPairs(vector<KeyPoint>& keypoints1, Mat& descriptors1,
               vector<KeyPoint>& keypoints2, Mat& descriptors2,
               vector<Point2f>& srcPoints, vector<Point2f>& dstPoints) {
  for (int i = 0; i < descriptors1.rows; i++) {
    KeyPoint pt1 = keypoints1[i];
    Mat desc1 = descriptors1.row(i);
    int nn = nearestNeighbor(desc1, keypoints2, descriptors2);
    if (nn >= 0) {
      KeyPoint pt2 = keypoints2[nn];
      srcPoints.push_back(pt1.pt);
      dstPoints.push_back(pt2.pt);
    }
  }
}

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 

//this function automatically gets rid of points for which tracking fails

  vector<float> err;					
  Size winSize=Size(21,21);																								
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
  cout <<"P1"<<points1.size() <<" " <<points2.size() <<endl<<endl;
  
  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}

     }
  cout <<"P1"<<points1.size() <<" " <<points2.size() <<endl<<endl;
  
}
