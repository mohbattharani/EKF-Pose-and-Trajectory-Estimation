/*
Connect with ardrone: rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=False _navdata_demo:=0
cd ROSPath >> catkin_make
>> source devel/setup.bash
>> rosrun ardrone_track ardrone_ekf  //Run efk exe
>> cd rosbagfilePath
>> rosbag play BagfileName.bag --clock --r 0.3  //run bag file at slow rate as vision processing takes time. 
*/
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include "eigen3/Eigen/Dense" // Matrix Algebra Library
#include <ardrone_autonomy/Navdata.h>

#include "ardrone.h"

typedef Eigen::MatrixXd Matrix;
using namespace std;

const double wallDistance = 0.2; // Distance from wall
geometry_msgs::Twist pos_a, pos_m, pos_g, pos_imu, pos_vis, pos_est;
double V = 0;
double W = 0;
double previousTime, now, past, dt;
visualization_msgs::MarkerArray markers;
visualization_msgs::Marker marker;
ardrone_autonomy::Navdata navdata;
bool inital = false;

void callbackIMU(const sensor_msgs::ImuConstPtr imuData);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void navCallback(const ardrone_autonomy::NavdataPtr nav_msg);
void updateMarker(geometry_msgs::Twist, int, int);
void updateMarkers ();
double rad2deg(double rad);
double deg2rad(double deg);
double compensate_yaw(double, double, double);

void initilize ();
ardrone drone1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_ekf");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Subscriber sub = nh.subscribe("/ardrone/imu", 2, callbackIMU);
    ros::Publisher qPosition = nh.advertise<geometry_msgs::Twist>("/myQuadrotor/Position", 1);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
    image_transport::Subscriber subCam = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
    drone1.initialize();
    
    ros::Rate rate(50);
    
    while (ros::ok())
    {
      
        qPosition.publish(pos_est); // Publish the Position calculated for visulization
	updateMarkers();
	vis_pub.publish( markers );
        ros::spinOnce();
        rate.sleep();
    }

    // Send the command to stop the robot if CTRL+C is pressed
    return 0;
}

void callbackIMU(const sensor_msgs::ImuConstPtr imuData)
{
 drone1.callbackIMU(imuData); 
}
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  
   drone1.SfMKLT(msg);
   cout <<"Vision updated"<<endl; 
 }
//==================================================================== 

double rad2deg(double rad) {
	return rad * (180 / M_PI);	
}
double deg2rad(double deg) {
	return deg * (M_PI / 180);
}

void initilize (){
  pos_est.angular.x = 0;
  pos_est.angular.y = 0;
  pos_est.angular.z = 0;
  pos_est.linear.x = 0;
  pos_est.linear.y = 0;
  pos_est.linear.z = 0;
  previousTime = ros::Time::now().toSec();
  inital = true; 
}


//  updateMarker to send on Rviz 
double id = 0;
void updateMarkers () {
  
  pos_est.linear.x = drone1.X(0,0);
  pos_est.linear.y = drone1.X(1,0);
  pos_est.linear.z = drone1.X(2,0);
  pos_est.angular.x = drone1.X(3,0);
  pos_est.angular.y = drone1.X(4,0);
  pos_est.angular.z = drone1.X(5,0);
  updateMarker (pos_est, id++, 3);
  markers.markers.push_back(marker);  
}

void updateMarker (geometry_msgs::Twist Pose, int id, int color){
  
  marker.pose.orientation.x = drone1.Xest.q.x();
  marker.pose.orientation.y = drone1.Xest.q.y();
  marker.pose.orientation.z = drone1.Xest.q.z();
  marker.pose.orientation.w = drone1.Xest.q.w();
  marker.pose.position.x = drone1.Xest.p(0,0);
  marker.pose.position.y = drone1.Xest.p(1,0);
  marker.pose.position.z = drone1.Xest.p(2,0);
  
  cout <<"Estimated pose: "<< drone1.Xest.p(0,0) <<" "<< drone1.Xest.p(1,0)<<" "<< drone1.Xest.p(2,0)<< " "<<endl;
  cout <<marker.pose.orientation.x <<" "<<marker.pose.orientation.y<<" "<<marker.pose.orientation.z<<" "<<marker.pose.orientation.w<<endl<<endl;
  
  marker.header.frame_id = "ardrone_base_link";  
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.scale.x = 0.2;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Brightness of the arrow
  if (color == 1){
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  } 
  else if (color == 2){ 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  else {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
}

