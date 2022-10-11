#include <string>
#include<geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "roboteq_serial_driver/wheelRPM.h"

double width_robot = 0.35; 
double wheel_radius = 0.0475;
double vl = 0.0;
double vr = 0.0;
double w = 0.0;
double rpsL = 0.0;
double rpsR = 0.0;
double wl = 0.0;
double wr = 0.0;
double th = 0.0 ; 
double npr = 6000 ; 
const float pi=3.1416;
double roll, pitch, yaw;
int c_state = 0;
int l_state = 0;
int ispublished = 0;

/*void imu_callback(const geometry_msgs::Twist &yaw_msgs){
	th = yaw_msgs.angular.z ; 
}*/

geometry_msgs::PoseStamped estpose;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg_)
{
  	estpose.header.frame_id = "odom";
	estpose.header.stamp = msg_.header.stamp;
	estpose.pose.position.x = msg_.pose.pose.position.x;
	ROS_INFO_STREAM("x: " << estpose.pose.position.x);
	estpose.pose.position.y = msg_.pose.pose.position.y;
	estpose.pose.position.z = 0;
	tf::Quaternion q(0, 0, msg_.pose.pose.orientation.z, msg_.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	estpose.pose.orientation.x = 0;
	estpose.pose.orientation.y = 0;
	estpose.pose.orientation.z = yaw;
	estpose.pose.orientation.w = 0;
	ispublished++;
}

void WheelCallback(const geometry_msgs::Twist &left_wheel)
{
	geometry_msgs::Twist twist = left_wheel;
	double pulsesSecL= left_wheel.linear.x; 	
	rpsL= pulsesSecL /npr; 
	wl= (2*pi*rpsL);  
	vl = wl*wheel_radius; 

	geometry_msgs::Twist twist = right_wheel;	
	double pulsesSecR= right_wheel.linear.x;  
	rpsR= pulsesSecR /npr; 
	wr = (2*pi*rpsR); 
	vr = wr*wheel_radius; 
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Subscriber wheel_vel_sub = n.subscribe("/wheel_rpm", 1000, WheelCallback);
        ros::Subscriber pose_est_sub = n.subscribe("/initialpose", 1000, poseCallback);
	// initial position
	double x = 0.0; 
	double y = 0.0;
	double th = 0.0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(1000);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	while (ros::ok()) {
		ros::spinOnce();
		current_time = ros::Time::now(); 

		double dt = (current_time - last_time).toSec();
		double delta_x = (vl +vr)* cos(th)* dt/2;
		double delta_y = (vl+vr)*sin(th) * dt/2;
		double delta_th = ((vl-vr) / width_robot) * dt;
		
		if(l_state == c_state){
			x += delta_x;
			y += delta_y;
			th += delta_th;
			ROS_INFO_STREAM("angle: " << th);}
		else if (l_state != c_state){
			x = estpose.pose.position.x;
			y = estpose.pose.position.y;
			th = yaw;
			ROS_INFO_STREAM("angle: " << yaw);}

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromYaw(th);

		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		broadcaster.sendTransform(odom_trans);

		//filling the odometry


		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		/*odom.pose.covariance[0] = (1e-3);
		odom.pose.covariance[7] = (1e-3);
		odom.pose.covariance[14] = (1e-6);
		odom.pose.covariance[21] = (1e-6);
		odom.pose.covariance[28] = (1e-6);
  		odom.pose.covariance[35] = (1e-3);*/
		//velocity
		/*odom.twist.twist.linear.x = (vl+vr)/2;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = (vl-vr)/2*width_robot;*/
		/*odom.twist.covariance[0] = 1e-3;
		odom.twist.covariance[7] = 1e-3;
		odom.twist.covariance[14] = 1e-3;
		odom.twist.covariance[21] = 1e-3;
		odom.twist.covariance[35] = 1e-3;*/
		last_time = current_time;

		// publishing the odometry and the new tf
		//broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
		//ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
