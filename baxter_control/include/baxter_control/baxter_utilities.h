/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
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
 *   * Neither the name of CU Boulder nor the names of its
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

/**
 * \brief   Helper functions for controlling baxter
 * \author  Dave Coleman
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Msgs
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/AssemblyState.h>

namespace baxter_control
{

static const std::string BAXTER_STATE_TOPIC = "/robot/state";

// Needed?
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string PLANNING_GROUP_NAME = "both_arms";
static const std::string BASE_LINK = "base"; //"/base";
static const std::string EE_GROUP = "right_hand";
static const std::string EE_JOINT = "right_gripper_l_finger_joint";
static const std::string EE_PARENT_LINK = "right_wrist";


class BaxterUtilities
{
public:

  ros::Publisher pub_baxter_enable_;
  ros::Subscriber sub_baxter_state_;

  // Interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> group_;

  // Remember the last baxter state and time
  baxter_core_msgs::AssemblyStateConstPtr baxter_state_;
  ros::Time baxter_state_timestamp_;

  BaxterUtilities()
  {
    ros::NodeHandle nh;

    group_.reset(new move_group_interface::MoveGroup("both_arms"));

    // ---------------------------------------------------------------------------------------------
    // Advertise services
    pub_baxter_enable_ = nh.advertise<std_msgs::Bool>("/robot/set_super_enable",10);

    // ---------------------------------------------------------------------------------------------
    // Start the state subscriber
    sub_baxter_state_ = nh.subscribe<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC,
                         1, &BaxterUtilities::stateCallback, this);
  }

  bool checkCommunication()
  {
    // Wait for initial state to be recieved
    int count = 0;
    while( ros::ok() && baxter_state_timestamp_.toSec() == 0 )
    {
      if( count > 20 ) // 20 is an arbitrary number for when to assume no state is being published
      {
        ROS_ERROR_STREAM_NAMED("utilities","No state message has been recieved on topic " 
          << BAXTER_STATE_TOPIC);
        return false;
      }

      ++count;
      ros::Duration(0.05).sleep();
    }

    return true;
  }

  bool checkReady()
  {
    // Check we have a recent state
    if(ros::Time::now() > baxter_state_timestamp_ + ros::Duration(1.0)) // check that the message timestamp is no older than 1 second
    {
      ROS_ERROR_STREAM_NAMED("utilities","Baxter state expired. State: \n" << *baxter_state_ );
      return false;
    }

    // Check for error
    if( baxter_state_->error == true )
    {
      ROS_ERROR_STREAM_NAMED("utilities","Baxter has an error :(  State: \n" << *baxter_state_ );
      ROS_WARN_STREAM_NAMED("temp","Temporarily ignoring error bit");
      // \todo return false;
    }    

    // Check for estop
    if( baxter_state_->stopped == true )
    {
      std::string estop_button;
      switch( baxter_state_->estop_button )
      {
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED:
        estop_button = "Robot is not stopped and button is not pressed";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_PRESSED:
        estop_button = "Pressed";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNKNOWN:
        estop_button = "STATE_UNKNOWN when estop was asserted by a non-user source";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_RELEASED:
        estop_button = "Was pressed, is now known to be released, but robot is still stopped.";
        break;
      default:
        estop_button = "Unkown button state code";
      }

      std::string estop_source;
      switch( baxter_state_->estop_source )
      {
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE:
        estop_source = "e-stop is not asserted";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_USER:
        estop_source = "e-stop source is user input (the red button)";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN:
        estop_source = "e-stop source is unknown";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_FAULT:
        estop_source = "MotorController asserted e-stop in response to a joint fault";
        break;
      case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_BRAIN:
        estop_source = "MotorController asserted e-stop in response to a lapse of the brain heartbeat";
        break;
      default:
        estop_source = "Unkown button source code";

      }

      ROS_ERROR_STREAM_NAMED("utilities","ESTOP Button: '" << estop_button << "'. Source: '" << estop_source << "'. State: \n" << *baxter_state_ );
      return false;
    }

    return true;
  }

  void stateCallback(const baxter_core_msgs::AssemblyStateConstPtr& msg)
  {
    baxter_state_ = msg;
    baxter_state_timestamp_ = ros::Time::now();
  }

  bool setPlanningGroup()
  {
    //std::string group_name = group_->getName();

    // Create MoveGroup for both arms
    group_.reset(new move_group_interface::MoveGroup("both_arms"));
  }

  bool enableBaxter()
  {
    ROS_INFO_STREAM_NAMED("utility","Enabling Baxter");

    // Check communication
    if( !checkCommunication() )
      return false;

    // Check for errors / estop
    if( !checkReady() )
      return false;

    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    pub_baxter_enable_.publish(enable_msg);
    ros::Duration(1.0).sleep();


    // Check it enabled
    int count = 0;
    while( ros::ok() && baxter_state_->enabled == false )
    {
      if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
      {
        ROS_ERROR_STREAM_NAMED("utilities","Failed to enable Baxter");
        return false;
      }

      ++count;
      ros::Duration(0.05).sleep();
    }    


    return true;
  }

  bool disableBaxter()
  {
    ROS_INFO_STREAM_NAMED("utility","Disabling Baxter");
    std_msgs::Bool enable_msg;
    enable_msg.data = false;
    pub_baxter_enable_.publish(enable_msg);
    ros::Duration(0.5).sleep();

    // Check if Baxter disabled successfully
    // \todo

    return true;
  }

  bool positionBaxterReady()
  {
    return sendToPose("both_ready");     
  }

  bool positionBaxterNeutral()
  {
    return sendToPose("both_neutral"); 
  }

  /**
   * \brief Send baxter to a pose defined in the SRDF
   * \param pose_name - name of pose in SRDF
   * \return true if sucessful in planning and moving there
   */
  bool sendToPose(const std::string &pose_name)
  {
    //    setPlanningGroup();

    // Send to ready position
    ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm ready positions...");
    group_->setNamedTarget(pose_name); 
    bool result = group_->move();

    if( !result )
      ROS_ERROR_STREAM_NAMED("utilities","Failed to send Baxter to pose " << pose_name);

    return result;
  }

};

} //namespace

