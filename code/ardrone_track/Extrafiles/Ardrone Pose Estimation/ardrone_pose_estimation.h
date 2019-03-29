/*
 *  QUT cyphy AR.Drone pose estimation ROS node.
 *  Copyright (C) 2012, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef ARDRONE_POSE_ESTIMATION_H
#define ARDRONE_POSE_ESTIMATION_H

#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <ardrone_pose_estimation/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cyphy_kbd_drone/KeyCode.h>
#include <std_msgs/Empty.h>
#include <ardrone_pose_estimation/Pose.h>
#include "ros/param.h"
#include "Kalman.h"
#include <dynamic_reconfigure/server.h>
#include <ardrone_pose_estimation/onlinegainConfig.h>
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/String.h>



#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

// for rectangle path
#define KEYCODE_q 0x71 // 'q'
#define KEYCODE_w 0x77 // 'w'
#define KEYCODE_s 0x73 // 's'
#define KEYCODE_a 0x61 // 'a'

// for triangle path
#define KEYCODE_d 0x64 // 'd'
#define KEYCODE_f 0x66 // 'f'
#define KEYCODE_r 0x72 // 'r'
#define KEYCODE_e 0x65 // 'e'
#define KEYCODE_v 0x76 // 'v'
#define KEYCODE_t 0x74 // 't'
#define KEYCODE_9 0x39 // 't'
#define KEYCODE_c 0x63 // 'c'

#define KEYCODE_SPACEBAR 0x20 //to start
#define KEYCODE_ESC 0x1B //to finish

#define MAX_HEIGHT 1.8
#define MIN_HEIGHT 0.4
#define MAX_YAW 120 //degree
#define MIN_YAW -120 //degree
#define HEIGHT_TICK 0.4  //metre
#define DEFAULT_HEIGHT 0.8 
#define YAW_TICK 10 //degree
//#define SAMPLING_TIME 0.05 // 20Hz
#define SAMPLING_TIME 0.006 // 161Hz

enum
{ P_GAIN,I_GAIN,D_GAIN,I_MAX,I_MIN};





inline float max(float a, float b) { return a > b ? a : b; }
inline float min(float a, float b) { return a < b ? a : b; }


using namespace std;

class pose_esti
{
 // typedef pcl::PointXYZ           PointT;
  //typedef pcl::PointCloud<PointT> PointCloudT;

private:
    // **** ROS-related
    ros::NodeHandle nh_;
    ros::Subscriber nav_sub;
    ros::Subscriber kbdSubscriber;
    ros::Subscriber takeoff_sub;
    ros::Subscriber vslam_pose_sub;
    ros::Subscriber vslam_states_sub;
    tf::TransformListener tf_sub_;

    
    ros::Publisher  cmd_pub;
    ros::Publisher  cmd_yaw_pub;
    ros::Publisher  pose_pub;
    ros::Publisher  new_pose_pub;
    ros::Publisher  goal_pub;
    ros::Publisher  ptam_init_pub;

    std_msgs::Float32MultiArray goal_msg;
    
    
    ros::Time now,past,takeoff_start,median_now,median_past,yaw_now,yaw_past;
    bool init,take_off,pick_takeoff_time,pick_init_yaw,plus_2_minus,minus_2_plus;
    double init_yaw,past_measured_yaw,cur_measured_yaw;

    struct SE3_poseT
    {
      float x, y, z;
      float roll,pitch,yaw;
    };

    SE3_poseT cur_pose, prev_pose,goal_pose;
    double PTAM_Yaw;

    std::string world_frame;
    //std::string base_frame;
    std::string imu_frame;
    std::string base_kf_frame;
    
    tf::TransformBroadcaster tf_broadcaster;

    control_toolbox::Pid pid_yaw;
		control_toolbox::Pid pid_pos_x;
		control_toolbox::Pid pid_pos_y;
		control_toolbox::Pid pid_z;
		control_toolbox::Pid pid_vel_x;
		control_toolbox::Pid pid_vel_y;

    double yaw_test;

    geometry_msgs::Twist cmd_vel,cmd_yaw;
    ardrone_pose_estimation::Pose pose,new_pose;
    double ts;// = now.toSec() - past.toSec();
    tf::StampedTransform worldToPTAMFixedTF;

    Kalman kalman;
    Matrix < float, 2, 1 > measure_vel;
    Matrix < float, 2, 1 > median_vel;
    Matrix < float, 4, 1 > measure;
    Matrix<float, 4, 1> esti;
    FILE* fp_vel;
    bool median_flag;
    bool invertable;
    bool ptam_init_commanded;
    bool pick_ptam_yaw_init_time;
    dynamic_reconfigure::Server<ardrone_pose_estimation::onlinegainConfig> srv;

    double pos_gain[5];
		double vel_gain[5];
    geometry_msgs::PoseWithCovarianceStamped ptam_pose;
    ardrone_pose_estimation::Navdata navdata;
    int ptam_init_try_cnt;
    std_msgs::String vslam_states;



    //void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void navCallback(const ardrone_pose_estimation::NavdataConstPtr& nav_msg);
    //void z_ctrl(double height);
    void kbdCallback(const cyphy_kbd_drone::KeyCode& msg);
    double compensate_yaw(double measured_yaw,double init_yaw);

    
    void publish_pose(double x,double y,double z, double roll,double pitch,double yaw);
    void publish_new_pose(double x,double y,double z, double roll,double pitch,double yaw);
    void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
    void landCallback(const std_msgs::Empty &msg);
    void resetCallback(const std_msgs::Empty &msg);
    void takeoffCallback(const std_msgs::Empty &msg);
    void vslam_poseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr &msg);
    void reconfigureCB(ardrone_pose_estimation::onlinegainConfig &config, uint32_t level);
    void vslam_statesCallback(const std_msgs::String &msg);
		void publish_kf_transform();

    double deg2rad(double deg);
    double rad2deg(double rad);
    
 public:
    pose_esti (ros::NodeHandle nh);
    virtual ~ pose_esti ();
};






#endif
