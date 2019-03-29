/*
 * Copyright 2017 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef VISION_H
#define VISION_H

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

class vision
{
public:
  vision();
  ~vision();
// Match feature points using ratio and symmetry test
  void robustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                      std::vector<cv::KeyPoint>& keypoints_frame,
                      const cv::Mat& descriptors_model );
  
  
  vector<cv::KeyPoint> keypointsNew, keypointsOld;
  Mat descriptorsOld, descriptorsNew;
  vector<Point2f> pointsNew, pointsOld;
  
  Mat cam_matrix;
  Mat dist_coeff;
  bool needtoinit;
  
  Mat rgbFrame, grayFrameNew, grayFrameOld;
  Mat opticalFlow;
  Mat Rc_rec, Tc_rec;
  Mat gP3, iP3;  //3D feature postion in global and IMU frames

private:
  int ratioTest(std::vector<std::vector<cv::DMatch> > &matches);

  // Insert symmetrical matches in symMatches vector
  void symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                     const std::vector<std::vector<cv::DMatch> >& matches2,
                     std::vector<cv::DMatch>& symMatches );

  
  
  FeatureDetector* detector;
  DescriptorExtractor* extractor;
  //FlannBasedMatcher matcher;
  vector<DMatch>matches;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  

};

#endif // VISION_H
