/*********************************************************************
# Copyright (c) 2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Hariharasudan Malaichamee
 *  \desc   Node that lies on the top and controls the robot based on the enable, disable, stop and reset commands
 */

#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "baxter_core_msgs/AssemblyState.h"
#include "baxter_core_msgs/EndEffectorState.h"
#include "baxter_core_msgs/EndEffectorProperties.h"

namespace baxter_en{

//Topics to subscribe and publish
static const std::string BAXTER_STATE_TOPIC = "/robot/state";
static const std::string BAXTER_ENABLE_TOPIC = "/robot/set_super_enable";
static const std::string BAXTER_STOP_TOPIC = "/robot/set_super_stop";
static const std::string BAXTER_RESET_TOPIC = "/robot/set_super_reset";

static const std::string BAXTER_LEFT_GRIPPER_ST = "/robot/end_effector/left_gripper/state";
static const std::string BAXTER_RIGHT_GRIPPER_ST = "/robot/end_effector/right_gripper/state";
static const std::string BAXTER_LEFT_GRIPPER_PROP = "/robot/end_effector/left_gripper/properties";
static const std::string BAXTER_RIGHT_GRIPPER_PROP = "/robot/end_effector/right_gripper/properties";

ros::Subscriber enable_sub_;
ros::Subscriber stop_sub_;
ros::Subscriber reset_sub_;
ros::Publisher assembly_state_pub_;
ros::Publisher left_grip_st_pub_;
ros::Publisher right_grip_st_pub_;
ros::Publisher left_grip_prop_pub_;
ros::Publisher right_grip_prop_pub_;

baxter_core_msgs::AssemblyState assembly_state_;
baxter_core_msgs::EndEffectorState left_grip_st;
baxter_core_msgs::EndEffectorState right_grip_st;
baxter_core_msgs::EndEffectorProperties left_grip_prop;
baxter_core_msgs::EndEffectorProperties right_grip_prop;

ros::Timer timer_;

/**
 * Method to enable the robot
 */
void enable(const std_msgs::Bool &msg)
{

	if (msg.data){
	  assembly_state_.enabled=true;
	}

	else {
	  assembly_state_.enabled=false;
	}
	assembly_state_.stopped=false;
	assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
	assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
}

/**
 * Method to stop the robot and capture the source of the stop
 */
void stop(const std_msgs::Empty &msg)
{
	assembly_state_.enabled=false;
	assembly_state_.stopped=true;
	assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
	assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_BRAIN;
}

/**
 * Method resets all the values to False and 0s
 */
void reset(const std_msgs::Empty &msg)
{
	assembly_state_.enabled=false;
	assembly_state_.stopped=false;
	assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
	assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
}

}//namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_enable");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  //Default values for the assembly state
  baxter_en::assembly_state_.enabled = false;             // true if enabled
  baxter_en::assembly_state_.stopped = false;            // true if stopped -- e-stop asserted
  baxter_en::assembly_state_.error = false;              // true if a component of the assembly has an error
  baxter_en::assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;      // button status
  baxter_en::assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;     // If stopped is true, the source of the e-stop.

  //Default values for the left and right gripper end effector states
  baxter_en::left_grip_st.timestamp.sec=0;
  baxter_en::left_grip_st.timestamp.nsec=0;
  baxter_en::left_grip_st.id=1;
  baxter_en::left_grip_st.enabled=1;
  baxter_en::left_grip_st.calibrated=1;
  baxter_en::left_grip_st.ready=1;
  baxter_en::left_grip_st.moving=0;
  baxter_en::left_grip_st.gripping=0;
  baxter_en::left_grip_st.missed=0;
  baxter_en::left_grip_st.error=0;
  baxter_en::left_grip_st.position=0.0;
  baxter_en::left_grip_st.force=0.0;
  baxter_en::left_grip_st.state="sample";
  baxter_en::left_grip_st.command="no_op";
  baxter_en::left_grip_st.command_sender="";
  baxter_en::left_grip_st.command_sequence=0;

  baxter_en::right_grip_st=baxter_en::left_grip_st; // Sample values recorded on both the grippers to do the spoof


  //Default values for the left and the right gripper properties
  baxter_en::left_grip_prop.id=65664;
  baxter_en::left_grip_prop.ui_type=2;
  baxter_en::left_grip_prop.manufacturer="test";
  baxter_en::left_grip_prop.product="test";
  baxter_en::left_grip_prop.product="test";
  baxter_en::left_grip_prop.hardware_rev="test";
  baxter_en::left_grip_prop.firmware_rev="test";
  baxter_en::left_grip_prop.firmware_date="test";
  baxter_en::left_grip_prop.controls_grip=true;
  baxter_en::left_grip_prop.senses_grip=true;
  baxter_en::left_grip_prop.reverses_grip=true;
  baxter_en::left_grip_prop.controls_force=true;
  baxter_en::left_grip_prop.senses_force=true;
  baxter_en::left_grip_prop.controls_position=true;
  baxter_en::left_grip_prop.senses_position=true;
  baxter_en::left_grip_prop.properties="";

  baxter_en::right_grip_prop=baxter_en::left_grip_prop; // Sample values recorded on both the grippers to do the spoof

  baxter_en::assembly_state_pub_ = n.advertise<baxter_core_msgs::AssemblyState>(baxter_en::BAXTER_STATE_TOPIC,1);
  baxter_en::left_grip_st_pub_ = n.advertise<baxter_core_msgs::EndEffectorState>(baxter_en::BAXTER_LEFT_GRIPPER_ST,1);
  baxter_en::right_grip_st_pub_ = n.advertise<baxter_core_msgs::EndEffectorState>(baxter_en::BAXTER_RIGHT_GRIPPER_ST,1);
  baxter_en::left_grip_prop_pub_ = n.advertise<baxter_core_msgs::EndEffectorProperties>(baxter_en::BAXTER_LEFT_GRIPPER_PROP,1);
  baxter_en::right_grip_prop_pub_ = n.advertise<baxter_core_msgs::EndEffectorProperties>(baxter_en::BAXTER_RIGHT_GRIPPER_PROP,1);

  baxter_en::enable_sub_=n.subscribe(baxter_en::BAXTER_ENABLE_TOPIC,100,baxter_en::enable);
  baxter_en::stop_sub_=n.subscribe(baxter_en::BAXTER_STOP_TOPIC,100,baxter_en::stop);
  baxter_en::reset_sub_=n.subscribe(baxter_en::BAXTER_RESET_TOPIC,100,baxter_en::reset);

  while (ros::ok())
  {
	  baxter_en::assembly_state_pub_.publish(baxter_en::assembly_state_);
	  baxter_en::left_grip_st_pub_.publish(baxter_en::left_grip_st);
	  baxter_en::right_grip_st_pub_.publish(baxter_en::right_grip_st);
	  baxter_en::left_grip_prop_pub_.publish(baxter_en::left_grip_prop);
	  baxter_en::right_grip_prop_pub_.publish(baxter_en::right_grip_prop);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
