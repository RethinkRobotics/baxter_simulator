/*********************************************************************
 * Software License Agreement (BSD License)
 *
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/* Author: Hariharasudan Malaichamee
 * Author: Dave Coleman
 * Desc:   Customized the default gazebo_ros_control_plugin.cpp
 */

// Overload the default plugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

// Controller Manager
#include <controller_manager_msgs/SwitchController.h>

// Baxter
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/AssemblyState.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

namespace baxter_gazebo_plugin
{
class BaxterGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin
{
private:
  ros::Subscriber left_command_mode_sub_;
  ros::Subscriber right_command_mode_sub_;
  ros::Subscriber robot_state_sub_;
  ros::Subscriber head_state_sub;
  ros::Subscriber left_gripper_state_sub;
  ros::Subscriber right_gripper_state_sub;

  // Rate to publish assembly state
  ros::Timer timer_;

  // Cache the message
  baxter_core_msgs::AssemblyState assembly_state_;

  // use this as flag to indicate current mode
  baxter_core_msgs::JointCommand right_command_mode_, left_command_mode_;

  boost::mutex mtx_;  // mutex for re-entrent calls to modeCommandCallback
  bool enable_cmd, is_disabled, head_is_started, left_gripper_is_started,
      right_gripper_is_started;  // enabled tracks the current status of the robot that is being published & is_disabled
                                 // keeps track of the action taken

public:
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    // Load parent class first
    GazeboRosControlPlugin::Load(parent, sdf);

    // Baxter customizations:
    ROS_INFO_STREAM_NAMED("ros_control_plugin", "Loading Baxter specific simulation components");

    // Subscribe to a topic that switches' Baxter's msgs
    left_command_mode_sub_ = model_nh_.subscribe<baxter_core_msgs::JointCommand>(
        "/robot/limb/left/joint_command", 1, &BaxterGazeboRosControlPlugin::leftModeCommandCallback, this);
    right_command_mode_sub_ = model_nh_.subscribe<baxter_core_msgs::JointCommand>(
        "/robot/limb/right/joint_command", 1, &BaxterGazeboRosControlPlugin::rightModeCommandCallback, this);
    head_state_sub = model_nh_.subscribe<baxter_core_msgs::HeadPanCommand>(
        "/robot/head/command_head_pan", 1, &BaxterGazeboRosControlPlugin::headCommandCallback, this);
    left_gripper_state_sub = model_nh_.subscribe<baxter_core_msgs::EndEffectorCommand>(
        "/robot/end_effector/left_gripper/command", 1, &BaxterGazeboRosControlPlugin::leftEndEffectorCommandCallback,
        this);
    right_gripper_state_sub = model_nh_.subscribe<baxter_core_msgs::EndEffectorCommand>(
        "/robot/end_effector/right_gripper/command", 1, &BaxterGazeboRosControlPlugin::rightEndEffectorCommandCallback,
        this);

    // Subscribe to the topic that publishes the robot's state
    robot_state_sub_ = model_nh_.subscribe<baxter_core_msgs::AssemblyState>(
        "/robot/state", 1, &BaxterGazeboRosControlPlugin::enableCommandCallback, this);

    enable_cmd = false;
    is_disabled = false;
    right_command_mode_.mode = -1;
    left_command_mode_.mode = -1;
    head_is_started = false;
    left_gripper_is_started = false;
    right_gripper_is_started = false;
  }

  void enableCommandCallback(const baxter_core_msgs::AssemblyState msg)
  {
    enable_cmd = msg.enabled;
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;

    // Check if we got disable signal and if the controllers are not already disabled
    if (!enable_cmd && !is_disabled)
    {
      stop_controllers.push_back("left_joint_effort_controller");
      stop_controllers.push_back("left_joint_velocity_controller");
      stop_controllers.push_back("left_joint_position_controller");
      stop_controllers.push_back("right_joint_effort_controller");
      stop_controllers.push_back("right_joint_velocity_controller");
      stop_controllers.push_back("right_joint_position_controller");
      stop_controllers.push_back("head_position_controller");
      stop_controllers.push_back("left_gripper_controller");
      stop_controllers.push_back("right_gripper_controller");

      if (!controller_manager_->switchController(start_controllers, stop_controllers,
                                                 controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
      {
        ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
      }
      else
      {
        // Resetting the command modes to the initial configuration
        ROS_INFO_NAMED("ros_control_plugin", "Robot is disabled");
        ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned off");
        right_command_mode_.mode = -1;
        left_command_mode_.mode = -1;
        head_is_started = false;
        left_gripper_is_started = false;
        right_gripper_is_started = false;
        is_disabled = true;
      }
    }
  }

  void leftEndEffectorCommandCallback(const baxter_core_msgs::EndEffectorCommand msg)
  {
    if (!left_gripper_is_started && enable_cmd)
    {
      std::vector<std::string> start_controllers;
      std::vector<std::string> stop_controllers;

      start_controllers.push_back("left_gripper_controller");
      if (!controller_manager_->switchController(start_controllers, stop_controllers,
                                                 controller_manager_msgs::SwitchController::Request::STRICT))
      {
        ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
      }
      else
      {
        ROS_INFO_NAMED("ros_control_plugin", "Robot is enabled");
        ROS_INFO_NAMED("ros_control_plugin", "Left Grippercontroller was successfully started");
        ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned on");
        left_gripper_is_started = true;
        is_disabled = false;
      }
    }
    else
      return;
  }
  void rightEndEffectorCommandCallback(const baxter_core_msgs::EndEffectorCommand msg)
  {
    if (!right_gripper_is_started && enable_cmd)
    {
      std::vector<std::string> start_controllers;
      std::vector<std::string> stop_controllers;

      start_controllers.push_back("right_gripper_controller");
      if (!controller_manager_->switchController(start_controllers, stop_controllers,
                                                 controller_manager_msgs::SwitchController::Request::STRICT))
      {
        ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
      }
      else
      {
        ROS_INFO_NAMED("ros_control_plugin", "Robot is enabled");
        ROS_INFO_NAMED("ros_control_plugin", "Right Grippercontroller was successfully started");
        ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned on");
        right_gripper_is_started = true;
        is_disabled = false;
      }
    }
    else
      return;
  }
  void headCommandCallback(const baxter_core_msgs::HeadPanCommand msg)
  {
    if (!head_is_started && enable_cmd)
    {
      std::vector<std::string> start_controllers;
      std::vector<std::string> stop_controllers;

      start_controllers.push_back("head_position_controller");
      if (!controller_manager_->switchController(start_controllers, stop_controllers,
                                                 controller_manager_msgs::SwitchController::Request::STRICT))
      {
        ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
      }
      else
      {
        ROS_INFO_NAMED("ros_control_plugin", "Robot is enabled");
        ROS_INFO_NAMED("ros_control_plugin", "Head controller was successfully started");
        ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned on");
        head_is_started = true;
        is_disabled = false;
      }
    }
    else
      return;
  }
  void leftModeCommandCallback(const baxter_core_msgs::JointCommandConstPtr& msg)
  {
    // Check if we already received this command for this arm and bug out if so
    if (left_command_mode_.mode == msg->mode)
    {
      return;
    }
    else if (enable_cmd)
    {
      left_command_mode_.mode = msg->mode;  // cache last mode
      modeCommandCallback(msg, "left");
    }
    else
    {
      ROS_WARN_STREAM_NAMED("ros_control_plugin", "Enable the robot");
      return;
    }
  }

  void rightModeCommandCallback(const baxter_core_msgs::JointCommandConstPtr& msg)
  {
    // Check if we already received this command for this arm and bug out if so
    if (right_command_mode_.mode == msg->mode)
    {
      return;
    }
    else if (enable_cmd)
    {
      right_command_mode_.mode = msg->mode;  // cache last mode
      modeCommandCallback(msg, "right");
    }
    else
    {
      ROS_WARN_STREAM_NAMED("ros_control_plugin", "Enable the robot");
      return;
    }
  }

  void modeCommandCallback(const baxter_core_msgs::JointCommandConstPtr& msg, const std::string& side)
  {
    ROS_DEBUG_STREAM_NAMED("ros_control_plugin", "Switching command mode for side " << side);

    // lock out other thread(s) which are getting called back via ros.
    boost::lock_guard<boost::mutex> guard(mtx_);

    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    std::string start, stop;

    switch (msg->mode)
    {
      case baxter_core_msgs::JointCommand::POSITION_MODE:
      case baxter_core_msgs::JointCommand::RAW_POSITION_MODE:
        start_controllers.push_back(side + "_joint_position_controller");
        stop_controllers.push_back(side + "_joint_velocity_controller");
        stop_controllers.push_back(side + "_joint_effort_controller");
        start = side + "_joint_position_controller";
        stop = side + "_joint_velocity_controller and " + side + "_joint_effort_controller";
        break;
      case baxter_core_msgs::JointCommand::VELOCITY_MODE:
        start_controllers.push_back(side + "_joint_velocity_controller");
        stop_controllers.push_back(side + "_joint_position_controller");
        stop_controllers.push_back(side + "_joint_effort_controller");
        start = side + "_joint_velocity_controller";
        stop = side + "_joint_position_controller and " + side + "_joint_effort_controller";
        break;
      case baxter_core_msgs::JointCommand::TORQUE_MODE:
        start_controllers.push_back(side + "_joint_effort_controller");
        stop_controllers.push_back(side + "_joint_position_controller");
        stop_controllers.push_back(side + "_joint_velocity_controller");
        start = side + "_joint_position_controller";
        stop = side + "_joint_velocity_controller and " + side + "_joint_velocity_controller";
        break;
      default:
        ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Unknown command mode " << msg->mode);
        return;
    }
    // Checks if we have already disabled the controllers
    /** \brief Switch multiple controllers simultaneously.
     *
     * \param start_controllers A vector of controller names to be started
     * \param stop_controllers A vector of controller names to be stopped
     * \param strictness How important it is that the requested controllers are
     * started and stopped.  The levels are defined in the
     * controller_manager_msgs/SwitchControllers service as either \c BEST_EFFORT
     * or \c STRICT.  \c BEST_EFFORT means that \ref switchController can still
     * succeed if a non-existant controller is requested to be stopped or started.
     */
    if (!controller_manager_->switchController(start_controllers, stop_controllers,
                                               controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    {
      ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
    }
    else
    {
      is_disabled = false;
      ROS_INFO_NAMED("ros_control_plugin", "Robot is enabled");
      ROS_DEBUG_STREAM_NAMED("ros_control_plugin", "Controller " << start << " started and " << stop << " stopped.");
      ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned on by mode command callback");
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BaxterGazeboRosControlPlugin);
}  // namespace
