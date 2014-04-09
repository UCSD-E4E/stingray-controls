<<<<<<< HEAD
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
=======
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
>>>>>>> e16b0364b493536517a3eec58e42b113aa4926d5
#include <iostream>				// std::cout
using namespace std;
#define pi 3.14159265359
#define dt .01

<<<<<<< HEAD
=======


// get the angle using the complementary filter as described in:
// http://www.pieter-jan.com/node/11
float ComplementaryFilter (float angle, float cnvtd_accdata)
{
	angle = 0.98*angle + 0.02*cnvtd_accdata;
	return angle;
}

// get angle from accelerometer data for use in the complementary filter
float acc2angle (float acc1, float acc2)
{
	float angle = 0;
	angle = atan2f((float)acc1, (float)acc2);
	return angle;
}

>>>>>>> e16b0364b493536517a3eec58e42b113aa4926d5
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
<<<<<<< HEAD
	// y = y + alpha * (x - y);
	y = alpha*y + (1-alpha)*x;
	// alternative:
	// make moving average filter - a form of low pass filtering
	// in image processing, done by: avg(n) = ((n-1)*avg(n-1)+input)/n
	// where n is the current iteration (number of images).
=======
	y = y + alpha * (x - y);
>>>>>>> e16b0364b493536517a3eec58e42b113aa4926d5
}
//---------------------------------------------------------------
void getPosition (float& la, float& la1, float& fltrd_acc, float& fltrd_acc_prev, float& vel, float& vel1, float& position)
{
<<<<<<< HEAD
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
=======
	// use high pass filter to filter out the dc offset
	if (abs(la)<0.2)
	{
		highpass(la1, la, fltrd_acc,0.7);
		la1 = la;
	}

	// otherwise use low pass filter to filter out noise
	else
	{	
		la1 = la;
		lowpass(la, fltrd_acc, .9);
	}

	// update values
	vel1 = vel;
	// get velocity
	vel = integrate(fltrd_acc_prev,fltrd_acc,vel);
	// vel = integrate(la1,la,vel);
	// get position
	position = integrate(vel1, vel, position);

	// when IMU is at rest (which still has some offset present) set the velocity to 0
	if (abs(fltrd_acc - fltrd_acc_prev) < 0.005 && abs(fltrd_acc) < 0.03)
	{
		vel = 0;
	}

	fltrd_acc_prev = fltrd_acc;
}

// use gyro data to filter out gravitational acceleration *still needs some tweaking
void compensateG (float& x, float& y, float& z, float& G, float roll, float pitch, float yaw)
{
	// assumption : gx+gy+gz = G
	float gx, gy, gz = 0;
	gy = sin(roll)*y;
	gx = sin(pitch)*x;
	gz = G - gx - gy;

	x = abs(x)-abs(gx);
	y = abs(y)-abs(gy);
	z = z-gz;
>>>>>>> e16b0364b493536517a3eec58e42b113aa4926d5
}


//initialize global parameters
float lax, lax1, lay, lay1, laz, laz1 = 0;
float x_fltrd_acc_prev, x_fltrd_acc, x_velocity, x_velocity1, x_position = 0;
float y_fltrd_acc_prev, y_fltrd_acc, y_velocity, y_velocity1, y_position = 0;
float z_fltrd_acc_prev, z_fltrd_acc, z_velocity, z_velocity1, z_position = 0;
<<<<<<< HEAD
float x_offset, y_offset, z_offset = 0;
int i, j = 0;
float q0,q1,q2,q3 = 0;
=======
float x_offset, y_offset, z_G = 0;
int i, j, avg_sum = 0;

float avx, avy, avz = 0;
float avx_prev, avy_prev, avz_prev = 0;
float x_angle, roll, y_angle, pitch, z_angle, yaw = 0;
float roll_acc_ang, pitch_acc_ang, yaw_acc_ang = 0;

>>>>>>> e16b0364b493536517a3eec58e42b113aa4926d5
//---------------------------------------------------



void imucallback(const sensor_msgs::Imu::ConstPtr& msg)
{
<<<<<<< HEAD
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
=======
	ROS_INFO("Y Position: %lf", y_position);
	ROS_INFO("Y velocity: %lf", y_velocity);
	ROS_INFO("Y Linear Acceleration: %lf", msg -> linear_acceleration.y);
	ROS_INFO("Y Filtered Acceleration : %lf", lay);
	ROS_INFO("pitch: %lf\n", pitch*180/pi);


	//store IMU data into temperary parameters
	lax = msg -> linear_acceleration.x;
	lay = msg -> linear_acceleration.y;
	// lay = 1.0000;
	laz = msg -> linear_acceleration.z;
	avx = msg -> angular_velocity.x;
	avy = msg -> angular_velocity.y;
	avz = msg -> angular_velocity.z;
	//----------------------------------------------------------------------
	// get angle
	x_angle = integrate(avx_prev, avx, x_angle);
	avx_prev = avx;
	roll_acc_ang = acc2angle(-avz,avy);
	roll = ComplementaryFilter(x_angle,roll_acc_ang);

	y_angle = integrate(avy_prev, avy, y_angle);
	avy_prev = avy;
	pitch_acc_ang = acc2angle(-avz,avx);
	pitch = ComplementaryFilter(y_angle,pitch_acc_ang);

	z_angle = integrate(avz_prev, avz, z_angle);
	avz_prev = avz;
	yaw_acc_ang = acc2angle(avx,avy);
	yaw = ComplementaryFilter(z_angle,yaw_acc_ang);
	//----------------------------------------------------------------------
	// get position
	// compensate the error in the accelerometer
	if (i<100)
	{
		i++;
	}
	else if(i==100)
	{
		x_offset = lax;
		y_offset = lay;
		z_G = laz;
		i++;
	}
	lax = lax-x_offset;
	lay = lay-y_offset;

	compensateG(lax,lay,laz,z_G,roll,pitch,yaw);

	// getPosition(lax,lax1,x_fltrd_acc,x_fltrd_acc_prev,x_velocity,x_velocity1,x_position);
	getPosition(lay,lay1,y_fltrd_acc,y_fltrd_acc_prev,y_velocity,y_velocity1,y_position);
	//----------------------------------------------------------------------

>>>>>>> e16b0364b493536517a3eec58e42b113aa4926d5
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("imu/data", 1000, imucallback);

<<<<<<< HEAD
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


=======
	cout << x_position << '\n';
	ros::spin();
>>>>>>> e16b0364b493536517a3eec58e42b113aa4926d5
	return 0;
}

