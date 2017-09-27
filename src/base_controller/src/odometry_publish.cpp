#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include "string.h"
#include <base_controller/base_Speed.h>

using namespace std;

float vx = 0.0;
float vy = 0.0;
float vth = 0.0;

void speed_to_OdomCallback(const  base_controller::base_Speed &speed_aux)
{
    base_controller::base_Speed speed = speed_aux;
	vx = speed_aux.linear_x;
	vy = speed_aux.linear_y;
	vth = speed_aux.angular_z;   
}



int main(int argc,char** argv)
{
    ros::init(argc,argv,"odometry_publish");
    ros::NodeHandle n;
    ros::Subscriber base_speed_sub = n.subscribe("base_controller_speed", 50, speed_to_OdomCallback);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  	tf::TransformBroadcaster odom_broadcaster;

	float x = 0.0;
	float y = 0.0;
	float th = 0.0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate loop_rate(10);
	while(n.ok())
	 {

		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		float dt = (current_time - last_time).toSec();
		float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		float delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;
		loop_rate.sleep();
	 }
	  
    return 0;

}

