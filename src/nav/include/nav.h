/**
 *  \file nav.h
 *  \brief ROS node for the navigation control algorithms.
 */

/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef NAV_H
#define NAV_H

// System libraries.
#include <cstdlib>

// ROS.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Bullet includes from ROS.
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

// Custom messages.
#include "os5000/DepthMessage.h"
#include "nav/ControlInputs.h"
#include "pid/PID.h"
#include "planner/TargetStates.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <nav/navParamsConfig.h>

using std::string;

/******************************
 * Classes
 *****************************/

class Nav
{
public:
    //! Constructor.
    Nav();

    //! Destructor.
    ~Nav();

    // Member Functions

    //! State initialization.
    void initStates();

    //! Publisher for control inputs
    void publishControlInputs(ros::Publisher *pub_control_inputs);

    //! Callback function for compass data
    void compassCallback(const sensor_msgs::Imu::ConstPtr& msg);

	void compassDepthCallback(const os5000::DepthMessage::ConstPtr& msg);
	
    //! Callback function for compass data
    void microstrainCallback(/*const tf::TranformListener& listener, */const sensor_msgs::Imu::ConstPtr& msg);

    //! Callback function for Target States
    void targetStatesCallback(const planner::TargetStates::ConstPtr& msg);

    //! Callback function for dynamic configuration of gains
    void configCallback(nav::navParamsConfig& config, uint32_t level);

    // States.
    bool roll_upsidedown;
    
    //! Roll state. In radians (-pi,pi], measured around longitudinal axis, positive is left wing up, negative is left wing down.
    double roll;
    //! Pitch state. In radians (-pi,pi], measured around lateral axis, positive is nose up, negative is nose down.
    double pitch;
    //! Yaw state. In radians (-pi,pi], measured around vertical axis, positive is left turn, negative is right turn.
    double yaw;
    //! Depth state. In meters below water surface. Positive is below water, negative is above water.
    double depth;
    //! Forward velocity. In meters/second.
    double surge;

    // Target states.
    //! Desired roll angle.
    double target_roll;
    //! Deisred pitch angle.
    double target_pitch;
    //! Desired yaw angle.
    double target_yaw;
    //! Desired depth.
    double target_depth;
    //! Desired forward velocity.
    double target_surge;

    // Previous state errors, used for calculating D error.
    //! Roll error from previous timestep.
    double prev_roll_error;
    //! Pitch error from previous timestep.
    double prev_pitch_error;
    //! Yaw error from previous timestep.
    double prev_yaw_error;
    //! Depth error from previous timestep.
    double prev_depth_error;
    //! Surge error from previous timestep.
    double prev_surge_error;

    // Previous integrator values, used for calculating I error.
    //! Sum of all prior roll errors.
    double prev_roll_int;
    //! Sum of all prior pitch errors.
    double prev_pitch_int;
    //! Sum of all prior yaw errors.
    double prev_yaw_int;
    //! Sum of all prior depth errors.
    double prev_depth_int;
    //! Sum of all prior surge errors.
    double prev_surge_int;

    // Variables for storing current integrator value.
    //! Current integrator value for roll.
    double curr_roll_int;
    //! Current integrator value for pitch.
    double curr_pitch_int;
    //! Current integrator value for yaw.
    double curr_yaw_int;
    //! Current integrator value for depth.
    double curr_depth_int;
    //! Current integrator value for surge.
    double curr_surge_int;
    //! Minimum integrator value allowed for roll.
    double min_int_roll;
    //! Maximum integrator value allowed for roll.
    double max_int_roll;
    //! Minimum integrator value allowed for pitch.
    double min_int_pitch;
    //! Maximum integrator value allowed for pitch.
    double max_int_pitch;
    //! Minimum integrator value allowed for yaw.
    double min_int_yaw;
    //! Maximum integrator value allowed for yaw.
    double max_int_yaw;
    //! Minimum integrator value allowed for depth.
    double min_int_depth;
    //! Maximum integrator value allowed for depth.
    double max_int_depth;
    //! Minimum integrator value allowed for surge.
    double min_int_surge;
    //! Maximum integrator value allowed for surge.
    double max_int_surge;

    // Actuator Limits
    //! Maximum angle attainable by yaw control surface;
    double max_yaw_angle;
    //! Minimum angle attainable by yaw control surface;
    double min_yaw_angle;

    // Variables for storing control outputs u.
    //! Output command for roll.
    double u_roll;
    //! Output command for pitch.
    double u_pitch;
    //! Output command for yaw.
    double u_yaw;
    //! Output command for depth.
    double u_depth;
    //! Output command for surge.
    double u_surge;

    // Control gains.
    //! Proportional gain for roll.
    double gain_roll_p;
    //! Integral gain for roll.
    double gain_roll_i;
    //! Differential gain for roll.
    double gain_roll_d;
    //! Proportional gain for pitch.
    double gain_pitch_p;
    //! Integral gain for pitch.
    double gain_pitch_i;
    //! Differential gain for pitch.
    double gain_pitch_d;
    //! Proportional gain for yaw.
    double gain_yaw_p;
    //! Integral gain for yaw.
    double gain_yaw_i;
    //! Differential gain for yaw.
    double gain_yaw_d;
    //! Proportional gain for depth.
    double gain_depth_p;
    //! Integral gain for depth.
    double gain_depth_i;
    //! Differential gain for depth.
    double gain_depth_d;
    //! Proportional gain for surge.
    double gain_surge_p;
    //! Integral gain for surge.
    double gain_surge_i;
    //! Differential gain for surge.
    double gain_surge_d;

    //alpha term for the derivative term of the PID controller
    double alpha_roll;
    double alpha_pitch;
    double alpha_yaw;
    double alpha_depth;
    double alpha_surge;


    //variables for storing derivative errors
    double prev_roll_deriv;
    double prev_pitch_deriv;
    double prev_yaw_deriv;
    double prev_depth_deriv;
    double prev_surge_deriv;
    
    // Delta time for each control loop.
    //! Elapsed time between control loops for roll.
    double dt_roll;
    //! Elapsed time between control loops for pitch.
    double dt_pitch;
    //! Elapsed time between control loops for yaw.
    double dt_yaw;
    //! Elapsed time between control loops for depth.
    double dt_depth;
    //! Elapsed time between control loops for surge.
    double dt_surge;
};

#endif // NAV_H
