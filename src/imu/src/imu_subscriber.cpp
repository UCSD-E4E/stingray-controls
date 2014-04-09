// Joseph Lee (jyonalee@gmail.com)
// Calculate dead reckoning from linear acceleration
// and quaternion values outputed by microstrain IMU.
// Subtract offset caused by Earth's gravity with the 
// quaternion heading values given by the IMU.
// Then take double integral to get position (displacement).
// This code will subscribe to imu_node (/imu/data), calculate displacement, 
// then publish the displacement to be used for further use (/dead_reckoning_chatter)
// such as the controlling motors for dead reckoning

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <imu/position.h>
#include <math.h>
#include <iostream>				// std::cout
using namespace std;
#define pi 3.14159265359
#define dt .01

// used trapezoidal rule to integrate
float integrate (float x1, float x2, float intgrX)
{
	float intgrout = 0;
	intgrout = intgrX + (x2+x1)*dt/2;
	return intgrout;
}

//-------------------------------------------------------------
void highpass (float& x_prev, float& x, float& y, float alpha)
{
	y = alpha*y + alpha*(x-x_prev);
}

void lowpass (float& x, float& y, float alpha)
{
	// y = y + alpha * (x - y);
	y = alpha*y + (1-alpha)*x;
	// alternative:
	// make moving average filter - a form of low pass filtering
	// in image processing, done by: avg(n) = ((n-1)*avg(n-1)+input)/n
	// where n is the current iteration (number of images).
}
//---------------------------------------------------------------
void getPosition (float& la, float& la1, float& fltrd_acc, float& fltrd_acc_prev, float& vel, float& vel1, float& position)
{
	// filter linear acceleration with lowpass filter
	lowpass(la, fltrd_acc, .7);
	// fltrd_acc = floor(fltrd_acc*1000+.5)/1000;

	// calculate velocity
	vel = integrate(fltrd_acc_prev,fltrd_acc,vel);
	// if
	// vel = integrate(la1,la,vel);

	// calculate position
	position = integrate(vel1, vel, position);

	// update variables
	vel1 = vel;
	la1 = la;
	fltrd_acc_prev = fltrd_acc;
	// fltrd_vel_prev = fltrd_vel;
}


//initialize global parameters
float lax, lax1, lay, lay1, laz, laz1 = 0;
float x_fltrd_acc_prev, x_fltrd_acc, x_velocity, x_velocity1, x_position = 0;
float y_fltrd_acc_prev, y_fltrd_acc, y_velocity, y_velocity1, y_position = 0;
float z_fltrd_acc_prev, z_fltrd_acc, z_velocity, z_velocity1, z_position = 0;
float x_offset, y_offset, z_offset = 0;
int i, j = 0;
float q0,q1,q2,q3 = 0;
//---------------------------------------------------



void imucallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	// for debugging purposes.
	// ROS_INFO("X Position: %lf", x_position);
	// ROS_INFO("X Velocity: %lf", x_velocity);
	// ROS_INFO("Y Position: %lf", y_position);
	// ROS_INFO("Y velocity: %lf", y_velocity);
	// ROS_INFO("X Linear Acceleration: %lf", msg -> linear_acceleration.x);
	// ROS_INFO("X lin acc : %lf", lax);
	// ROS_INFO("X lin accf: %lf", x_fltrd_acc);
	// ROS_INFO("Y Linear Acceleration: %lf", msg -> linear_acceleration.y);
	// ROS_INFO("Y compensated lin acc: %lf", lay);
	// ROS_INFO("Z Linear Acceleration: %lf", msg -> linear_acceleration.z);
	// ROS_INFO("Z compensated lin acc: %lf", laz);
	// ROS_INFO("Y Angular Velocity:    %lf", msg -> angular_velocity.y);
	// ROS_INFO("\n");

	// store IMU data into temperary parameters
	lax = msg -> linear_acceleration.x;
	lay = msg -> linear_acceleration.y;
	laz = msg -> linear_acceleration.z;
	// quaternion orientation used to take out the gravitational acceleration
	q0 = msg -> orientation.x;
	q1 = msg -> orientation.y;
	q2 = msg -> orientation.z;
	q3 = msg -> orientation.w;
	// calculations to compensate out gravitational acceleration
	x_offset = 2*((q1*q3)-(q0*q2))*(-9.8);
	y_offset = 2*((q2*q1)+(q0*q3))*(9.8);
	z_offset = (q2*q2-q1*q1-q0*q0+q3*q3)*(9.8);
	lax = lax - x_offset;
	lay = lay - y_offset;
	laz = laz - z_offset;

	// calculate displacement.
	getPosition(lax,lax1,x_fltrd_acc,x_fltrd_acc_prev,x_velocity,x_velocity1,x_position);
	getPosition(lay,lay1,y_fltrd_acc,y_fltrd_acc_prev,y_velocity,y_velocity1,y_position);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("imu/data", 1000, imucallback);

	ros::init(argc,argv, "talker");
	ros::Publisher dr_pub = n.advertise<imu::position>("dead_reckoning_chatter",1000);
	ros::Rate loop_rate(100);
	imu::position msg;
	while (ros::ok())
	{
		msg.position_x = x_position;
		msg.position_y = y_position;
		msg.position_z = z_position;
		dr_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();


	return 0;
}

