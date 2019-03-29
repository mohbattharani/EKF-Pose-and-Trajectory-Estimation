#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include "eigen3/Eigen/Dense" // Matrix Algebra Library
#include <ardrone_autonomy/Navdata.h>

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
void navCallback(const ardrone_autonomy::NavdataPtr nav_msg);
void updateMarker(geometry_msgs::Twist, int, int);
void updateMarkers ();
double rad2deg(double rad);
double deg2rad(double deg);
double compensate_yaw(double, double, double);
void initilize ();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quadrotor_odometery");
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe("/ardrone/imu", 2, callbackIMU);
    ros::Subscriber subNav = nh.subscribe("/ardrone/navdata",2, navCallback);
   // ros::Publisher cmdPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher qPosition = nh.advertise<geometry_msgs::Twist>("/myQuadrotor/Position", 1);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
    
    geometry_msgs::Twist cmd;
    ros::Rate rate(50);
    
    while (ros::ok())
    {
      
       // qPosition.publish(pos_est); // Publish the Position calculated for visulization
	updateMarkers();
	vis_pub.publish( markers );

        ros::spinOnce();
        rate.sleep();
    }

    // Send the command to stop the robot if CTRL+C is pressed
    cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;
   // cmdPub.publish(cmd);
    return 0;
}

 
//==================================================================== 

bool init_nav= false;
double preYaw, curYaw, compassYaw, initYaw;
void navCallback(const ardrone_autonomy::NavdataPtr nav_msg){
   double alpha = 0.8;
   now = ros::Time::now().toSec();
   if (!init_nav){
     preYaw = deg2rad(nav_msg->rotZ);
     initYaw = preYaw;
     init_nav = true;
     past = now;
     return;
   }
   dt = now - past;
   curYaw = deg2rad(nav_msg->rotZ);
   compassYaw = compensate_yaw(curYaw, initYaw, preYaw);
   pos_m.linear.z = nav_msg->altd;
   pos_m.linear.x += dt*(cos(compassYaw)*nav_msg->vx-sin(compassYaw)*nav_msg->vy);  //metre
   pos_m.linear.y += dt*(sin(compassYaw)*nav_msg->vx+cos(compassYaw)*nav_msg->vy);  //metre
   pos_m.angular.x = deg2rad (nav_msg->rotX);
   pos_m.angular.y = deg2rad (nav_msg->rotY);
   pos_m.angular.z =  (compassYaw);
   // From callbackIMU
   cout << "dt:"<<dt<<pos_m.linear.x <<" "<<pos_m.linear.y<<" "<<pos_m.linear.z<<" ++ ";
   cout << nav_msg->rotX <<" "<<nav_msg->rotY <<" "<<compassYaw<<endl<<endl;
   preYaw = curYaw;
   past = now;
   //cout<<pos_est;
}

// ===============================================================================
double ax,ay,az,roll,pitch,yaw, mx,my,mz, yaw_t0;
double vx, vy, vx0=0, vy0=0, vz0=0;

void callbackIMU(const sensor_msgs::ImuConstPtr imuData)
{
  
    if (!inital) {
     initilize ();  
     yaw_t0 = tf::getYaw(imuData.get()->orientation);
     //pos_m.angular.z = yaw_t0;
    }
    double newTime = ros::Time::now().toSec();
    double dTime = newTime - previousTime;
    double alpha = 0.8;
    geometry_msgs::Twist dPosition;
    Matrix M(3,3), ang(3,1),Eu_t0(3,1), Eu_t1(3,1), T_bi(3,3), g(3,1);
 
    //====================gyroscope=============================
    ang << imuData.get()->angular_velocity.x,imuData.get()->angular_velocity.y, 			           	     
    imuData.get()->angular_velocity.z;
    Eu_t0 << pos_imu.angular.x, pos_est.angular.y, pos_imu.angular.z;
    
    M << 1, sin(pos_imu.angular.x)*tan(pos_imu.angular.y), cos(pos_imu.angular.x)*tan(pos_imu.angular.x),
	 0, cos(pos_imu.angular.x)                        , -sin(pos_imu.angular.x),
	 0, sin(pos_imu.angular.x)/cos(pos_imu.angular.y), cos(pos_imu.angular.x)/cos(pos_imu.angular.y);
    Eu_t1 = Eu_t0 + (M*ang)*dTime;
    pos_g.angular.x = Eu_t1(0, 0);
    pos_g.angular.y = Eu_t1(1, 0);
    pos_g.angular.z = Eu_t1(2, 0);
    
    //=========== convert accelerometer data from body frame ============
    roll = pos_imu.angular.x;
    pitch = pos_imu.angular.y;
    yaw = pos_imu.angular.z;
    g << 0.0, 0.0, 9.8;    //gravity
    
    Eu_t0<<imuData.get()->linear_acceleration.x,imuData.get()->linear_acceleration.y,imuData.get()->linear_acceleration.z;
    
    // body frame to intertial frame transformation Matrix
    T_bi << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw), sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch),
    cos(pitch)*sin(yaw), cos(roll)*cos(yaw)+sin(roll)*sin(yaw)*sin(pitch), cos(roll)*sin(yaw)*sin(pitch)-cos(yaw)*cos(roll),
    -sin(pitch), 	cos(pitch)*sin(roll), 	cos(roll)*cos(pitch);
    
    Eu_t1 = T_bi*Eu_t0 + g;   //accelerometer values in intertial frame
    
    ax = Eu_t1(0, 0);
    ay = Eu_t1(1, 0);
    az = Eu_t1(2, 0);
     // Roll pitch angles from accelerometer
    vx0 += ax * dTime;
    vy0 += ay * dTime;
    vz0 += az * dTime;
    
    vx = ax * dTime;
    vy = ay * dTime;
    pos_a.angular.x = -atan(ay/az);
    pos_a.angular.y = -atan(ax/ sqrt(ay*ay + az*az));
    
    // Intergrate accelerometer values to measure Distances
    pos_a.linear.x += vx0 * dTime ;// * cos(pos_m.angular.z) - sin(pos_m.angular.z)* ay);
    pos_a.linear.y += vy0 * dTime ;
    pos_a.linear.z += vz0 * dTime ;
    
    
    // ========== magnetrometer ========================= 
//     pos_m.angular.z = tf::getYaw(imuData.get()->orientation);// compensate_yaw(tf::getYaw(imuData.get()->orientation), yaw_t0, pos_m.angular.z);
//     pos_m.linear.x += dTime * (cos(pos_m.angular.z) * vx - sin(pos_m.angular.z) * vy);  //metre
//     pos_m.linear.y += dTime * (sin(pos_m.angular.z) * vx + cos(pos_m.angular.z) * vy);  //metre
        
    // fusion
    pos_imu.angular.x = alpha * pos_g.angular.x + (1-alpha) * pos_a.angular.x;
    pos_imu.angular.y = alpha * pos_g.angular.y + (1-alpha) * pos_a.angular.y;
    pos_imu.angular.z = alpha * pos_g.angular.z + (1-alpha) * pos_m.angular.z;
    pos_imu.linear.x  = alpha * pos_a.linear.x  + (1-alpha) * pos_m.linear.x;
    pos_imu.linear.y  = alpha * pos_a.linear.y  + (1-alpha) * pos_m.linear.y;
    pos_imu.linear.z  = pos_a.linear.z;
    
    
    pos_est = pos_a;
   // cout << pos_m.angular.z <<" "<<Eu_t1(2, 0)<<"..."<<pos_imu.angular.z<<" " <<endl;
    previousTime = newTime;
}

// =========== compensate_yaw ==================================

// All input and output arguments are radian.
bool plus_2_minus, minus_2_plus;
double compensate_yaw(double measured_yaw,double init_yaw, double preYaw)
{
		double comp_yaw=0;;
		if((rad2deg(preYaw)>=179.9) && (rad2deg(measured_yaw)<=-179.9))
		plus_2_minus=true;

		if((rad2deg(preYaw)<=-179.9) && (rad2deg(measured_yaw)>=179.9))
		minus_2_plus=true;
		
		if(plus_2_minus) 
		  comp_yaw=deg2rad(360)+measured_yaw-init_yaw;
		else if(minus_2_plus) 
		  comp_yaw=measured_yaw-init_yaw-deg2rad(360);
		else
		  comp_yaw = measured_yaw-init_yaw;
	
		if(rad2deg(comp_yaw)>=179.9) 
		  comp_yaw = comp_yaw - deg2rad(360);
		else if(rad2deg(comp_yaw)<=-179.9) 
		  comp_yaw = deg2rad(360)+comp_yaw ;
		else 
		  comp_yaw = comp_yaw;
		
		plus_2_minus=0;
		minus_2_plus=0;
		return comp_yaw;
}

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
  //markers.markers.clear();
  /*updateMarker (pos_a, 1, 1);
  markers.markers.push_back(marker);
  updateMarker (pos_m, 2, 2);
  markers.markers.push_back(marker); */ 
  updateMarker (pos_m, id, 3);
  markers.markers.push_back(marker);  
}

void updateMarker (geometry_msgs::Twist Pose, int id, int color){
//  tf::Matrix3x3 obs_mat;
//   obs_mat.setEulerYPR(Position.angular.x, Position.angular.y, Position.angular.z);     
//   tf::Quaternion q;
//   obs_mat.getRotation(q);
 
  tf::Quaternion q = tf::createQuaternionFromRPY(Pose.angular.x, Pose.angular.y, Pose.angular.z);
  
  marker.pose.orientation.x = q.getX();
  marker.pose.orientation.y = q.getY();
  marker.pose.orientation.z = q.getZ();
  marker.pose.orientation.w = q.getW();
  marker.pose.position.x = Pose.linear.x;
  marker.pose.position.y = Pose.linear.y;
  marker.pose.position.z = Pose.linear.z;
  marker.header.frame_id = "map";  
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








