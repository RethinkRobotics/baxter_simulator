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

/* Author: Dave Coleman
   Desc:   Customized the default gazebo_ros_control_plugin.cpp
*/

// Overload the default plugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

// Controller Manager
#include <controller_manager_msgs/SwitchController.h>

// Baxter
#include <baxter_msgs/JointCommandMode.h>
#include <baxter_msgs/AssemblyState.h>


namespace baxter_gazebo_plugin
{

static const std::string BAXTER_STATE_TOPIC = "/sdk/robot/state";

class BaxterGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin
{
private:
  ros::Subscriber left_command_mode_sub_;
  ros::Subscriber right_command_mode_sub_;
  ros::Publisher assembly_state_pub_;

  // Rate to publish assembly state
  ros::Timer timer_;
  
  // Cache the message
  baxter_msgs::AssemblyState assembly_state_;

  // use this as flag to indicate current mode
  baxter_msgs::JointCommandMode right_command_mode_;
  baxter_msgs::JointCommandMode left_command_mode_;

  boost::mutex mtx_; // mutex for re-entrent calls to modeCommandCallback

public:

  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    // Load parent class first
    GazeboRosControlPlugin::Load(parent, sdf);

    // Baxter customizations:
    ROS_INFO_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Loading Baxter specific simulation components");

    // Subscribe to a topic that switches' Baxter's msgs
    left_command_mode_sub_ = nh_.subscribe<baxter_msgs::JointCommandMode>("/robot/limb/left/joint_command_mode",
                             1, &BaxterGazeboRosControlPlugin::leftModeCommandCallback, this);
    right_command_mode_sub_ = nh_.subscribe<baxter_msgs::JointCommandMode>("/robot/limb/right/joint_command_mode",
                             1, &BaxterGazeboRosControlPlugin::rightModeCommandCallback, this);

    // Start a publisher that publishes fake AssemblyState.msg data about Baxter
    assembly_state_pub_ = nh_.advertise<baxter_msgs::AssemblyState>(BAXTER_STATE_TOPIC,10);

    // Create assembly state message 
    assembly_state_.enabled = 1;             // true if enabled
    assembly_state_.stopped = 0;            // true if stopped -- e-stop asserted
    assembly_state_.error = 0;              // true if a component of the assembly has an error
    assembly_state_.estop_button = baxter_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;      // button status
    assembly_state_.estop_source = baxter_msgs::AssemblyState::ESTOP_SOURCE_NONE;     // If stopped is true, the source of the e-stop.  

    // Set publish frequency
    ros::NodeHandle nh_tilde("~");
    double publish_freq;
    nh_tilde.param("publish_frequency", publish_freq, 50.0);
    ros::Duration publish_interval = ros::Duration(1.0/std::max(publish_freq,1.0));

    // trigger to publish fixed joints
    timer_ = nh_tilde.createTimer(publish_interval, &BaxterGazeboRosControlPlugin::update, this);

    //preset the mode to illegal value
    right_command_mode_.mode = -1;
    left_command_mode_.mode = -1;
  }

  void update(const ros::TimerEvent& e)
  {
    assembly_state_pub_.publish(assembly_state_);
  }

  void leftModeCommandCallback(const baxter_msgs::JointCommandModeConstPtr& msg)
  {
    //Check if we already received this command for this arm and bug out if so
    if(left_command_mode_.mode == msg->mode)
    {
      return;
    }
    else
    {
      left_command_mode_.mode = msg->mode; //cache last mode
    }

    modeCommandCallback(msg, "left");
  }

  void rightModeCommandCallback(const baxter_msgs::JointCommandModeConstPtr& msg)
  {
    //Check if we already received this command for this arm and bug out if so
    if(right_command_mode_.mode == msg->mode)
    {
      return;
    }
    else
    {
      right_command_mode_.mode = msg->mode; //cache last mode
    }

    modeCommandCallback(msg, "right");
  }

  void modeCommandCallback(const baxter_msgs::JointCommandModeConstPtr& msg, const std::string& side)
  {
    ROS_DEBUG_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Switching command mode for side " 
      << side );
    
    //lock out other thread(s) which are getting called back via ros.
    boost::lock_guard<boost::mutex> guard(mtx_);

    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    switch(msg->mode)
    {
    case baxter_msgs::JointCommandMode::POSITION:
      start_controllers.push_back(side+"_joint_position_controller");
      stop_controllers.push_back(side+"_joint_velocity_controller");
      //stop_controllers.push_back(side+"_joint_effort_controller");
      break;
    case baxter_msgs::JointCommandMode::VELOCITY:
      start_controllers.push_back(side+"_joint_velocity_controller");
      stop_controllers.push_back(side+"_joint_position_controller");
      //stop_controllers.push_back(side+"_joint_effort_controller");
      break;
    case baxter_msgs::JointCommandMode::TORQUE:
      ROS_WARN_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Torque not implemented yet!");
      return;
      start_controllers.push_back(side+"_joint_effort_controller");
      stop_controllers.push_back(side+"_joint_position_controller");
      stop_controllers.push_back(side+"_joint_velocity_controller");
      break;
    default:
      ROS_ERROR_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Unknown command mode " << msg->mode);
      return;
    }

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
    if( !controller_manager_->switchController(start_controllers,stop_controllers, 
        controller_manager_msgs::SwitchController::Request::STRICT) )
    {
      ROS_ERROR_STREAM_NAMED("baxter_gazebo_ros_control_plugin","Failed to switch controllers");
    }
    
  }


};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BaxterGazeboRosControlPlugin);
} // namespace
