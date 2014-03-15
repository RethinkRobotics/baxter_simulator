/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, Open Source Robotics Foundation
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/*
 Author: Dave Coleman
 Contributors: Jonathan Bohren, Wim Meeussen, Vijay Pradeep
 Desc: Effort(force)-based position controller using basic PID loop
*/

#include <baxter_sim_controllers/baxter_gripper_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace baxter_sim_controllers {

BaxterGripperController::BaxterGripperController()
  : loop_count_(0)
{}

BaxterGripperController::~BaxterGripperController()
{
  calibrate_sub_.shutdown();
  position_sub_.shutdown();
  release_sub_.shutdown();
}

bool BaxterGripperController::init(hardware_interface::EffortJointInterface *robot, 
  ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) 
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  pid_controller_.reset(new control_toolbox::Pid());
  if (!pid_controller_->init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start the subscribers

  // in namespace robot/limb/right/accessory/gripper/
  ROS_DEBUG_STREAM_NAMED("gripper_controller","Starting calibration listener");
  calibrate_sub_ = n.subscribe<std_msgs::Empty>("command_calibrate",1, &BaxterGripperController::calibrateCB, this);

  ROS_DEBUG_STREAM_NAMED("gripper_controller","Starting close listener");
  position_sub_ = n.subscribe<std_msgs::Float32>("command_grip",1, &BaxterGripperController::positionCB, this);

  ROS_DEBUG_STREAM_NAMED("gripper_controller","Starting open listener");
  release_sub_ = n.subscribe<std_msgs::Empty>("command_release",1, &BaxterGripperController::releaseCB, this);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void BaxterGripperController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_->setGains(p,i,d,i_max,i_min);
}

void BaxterGripperController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_->getGains(p,i,d,i_max,i_min);
}

std::string BaxterGripperController::getJointName()
{
  return joint_.getName();
}

// Set the joint position command
void BaxterGripperController::setCommand(double cmd)
{
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(cmd);
}

void BaxterGripperController::starting(const ros::Time& time)
{
  command_.initRT(joint_.getPosition());
  pid_controller_->reset();
}

void BaxterGripperController::update(const ros::Time& time, const ros::Duration& period)
{
  double command = *(command_.readFromRT());

  double error, vel_error;

  // Compute position error
  if (joint_urdf_->type == urdf::Joint::REVOLUTE)
  {
    angles::shortest_angular_distance_with_limits(joint_.getPosition(),
      command,
      joint_urdf_->limits->lower,
      joint_urdf_->limits->upper,
      error);
  }
  else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(joint_.getPosition(), command);
  }
  else //prismatic
  {
    error = command - joint_.getPosition();
  }

  // Compute velocity error assuming desired velocity is 0
  vel_error = 0.0 - joint_.getVelocity();

  // Set the PID error and compute the PID command with nonuniform
  // time step size. This also allows the user to pass in a precomputed derivative error.
  double commanded_effort = pid_controller_->computeCommand(error, vel_error, period);

  /*
  static int i = 0;
  i++;
  if( i % 500 )
    ROS_INFO_STREAM_NAMED("update","command = " << command << ", effort = " << error);
  */
  joint_.setCommand(commanded_effort);


  // publish state
  if (loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command;
      controller_state_publisher_->msg_.process_value = joint_.getPosition();
      controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
        controller_state_publisher_->msg_.i,
        controller_state_publisher_->msg_.d,
        controller_state_publisher_->msg_.i_clamp,
        dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void BaxterGripperController::calibrateCB(const std_msgs::EmptyConstPtr& msg)
{
  setCommand(0.0);
}

void BaxterGripperController::positionCB(const std_msgs::Float32ConstPtr& msg)
{
  ROS_DEBUG_STREAM_NAMED("gripper_controller","data="<<msg->data);

  //setCommand(msg->data);
  
  // \todo remove this hack
  setCommand(-0.045);
}

void BaxterGripperController::releaseCB(const std_msgs::EmptyConstPtr& msg)
{
  setCommand(0.0);
}

} // namespace

PLUGINLIB_EXPORT_CLASS( baxter_sim_controllers::BaxterGripperController, controller_interface::ControllerBase)
