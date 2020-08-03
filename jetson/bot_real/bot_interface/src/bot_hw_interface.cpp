/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the bot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <bot_interface/bot_hw_interface.h>

namespace bot_control
{



botHWInterface::botHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("bot_hw_interface", "botHWInterface Ready.");
  //
  wheel_vel_pub = nh_.advertise<std_msgs::Float32MultiArray>("/vel_cmd", 10);
  wheel_vel_sub = nh_.subscribe("/vel_pose", 10,
                                &botHWInterface::read_vel, this);
}

void botHWInterface::read(ros::Duration &elapsed_time)
{



  ///*

  joint_velocity_[0] = curr_vel[0];
  joint_position_[0] += curr_vel[0]* elapsed_time.toSec();
  joint_velocity_[1] = curr_vel[1];
  joint_position_[1] += curr_vel[1]* elapsed_time.toSec();
  joint_velocity_[2] = curr_vel[2];
  joint_position_[2] += curr_vel[2]* elapsed_time.toSec();
  joint_velocity_[3] = curr_vel[3];
  joint_position_[3] += curr_vel[3]* elapsed_time.toSec();
  //*/


}

void botHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  vel_setpoint_cmd_array.data.clear();
  vel_setpoint_cmd_array.data.push_back(joint_velocity_command_[0]);
  vel_setpoint_cmd_array.data.push_back(joint_velocity_command_[1]);
  vel_setpoint_cmd_array.data.push_back(joint_velocity_command_[2]);
  vel_setpoint_cmd_array.data.push_back(joint_velocity_command_[3]);

  wheel_vel_pub.publish(vel_setpoint_cmd_array);


  /* // Uncomment For real robot To Publish to the Arduino Vel Setpoint Callback
  this -> cmd_to_setpoint();
  // Transform From joint_velocity_command to setpoint
  vel_setpoint_cmd_array.data.push_back(vel_set_cmd[0]);
  vel_setpoint_cmd_array.data.push_back(vel_set_cmd[1]);
  wheel_vel_pub.publish(vel_setpoint_cmd_array);

  // Clear Array
  vel_setpoint_cmd_array.data.clear();
  // Store Prev encoder ticks
  for (int i = 0; i < 4; i++) {
    prev_enc[i] = curr_enc[i];
  }
  //*/


}

void botHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}



void botHWInterface::read_vel(
    const std_msgs::Float32MultiArray::ConstPtr &vel_msg) {
  curr_vel[0] = vel_msg->data[2];
  curr_vel[1] = vel_msg->data[3];
  curr_vel[2] = vel_msg->data[0];
  curr_vel[3] = vel_msg->data[1];
}


}  // namespace
