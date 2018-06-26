/*********************************************************************
 # Copyright (c) 2013-2015, Rethink Robotics
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
 *  \desc   ROS node that publishes the states captured from the QT events
 */
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <baxter_sim_io/qnode.hpp>
#include <signal.h>

namespace baxter_sim_io
{
const std::string BAXTER_LEFTIL_TOPIC = "robot/digital_io/left_inner_light/state";
const std::string BAXTER_LEFTOL_TOPIC = "robot/digital_io/left_outer_light/state";
const std::string BAXTER_TORSO_LEFTIL_TOPIC = "robot/digital_io/torso_left_inner_light/state";
const std::string BAXTER_TORSO_LEFTOL_TOPIC = "robot/digital_io/torso_left_outer_light/state";
const std::string BAXTER_RIGHTIL_TOPIC = "robot/digital_io/right_inner_light/state";
const std::string BAXTER_RIGHTOL_TOPIC = "robot/digital_io/right_outer_light/state";
const std::string BAXTER_TORSO_RIGHTIL_TOPIC = "robot/digital_io/torso_right_inner_light/state";
const std::string BAXTER_TORSO_RIGHTOL_TOPIC = "robot/digital_io/torso_right_outer_light/state";

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
  init();
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void quit(int sig)
{
  ros::shutdown();
  exit(0);
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "baxter_sim_io");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  left_navigator = n.advertise<baxter_core_msgs::NavigatorState>("/robot/navigators/left_navigator/state", 1);
  right_navigator = n.advertise<baxter_core_msgs::NavigatorState>("/robot/navigators/right_navigator/state", 1);
  torso_left_navigator =
      n.advertise<baxter_core_msgs::NavigatorState>("/robot/navigators/torso_left_navigator/state", 1);
  torso_right_navigator =
      n.advertise<baxter_core_msgs::NavigatorState>("/robot/navigators/torso_right_navigator/state", 1);

  left_lower_cuff = n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 1);
  right_lower_cuff = n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/right_lower_cuff/state", 1);

  left_lower_button = n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_button/state", 1);
  right_lower_button = n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/right_lower_button/state", 1);

  left_upper_button = n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_upper_button/state", 1);
  right_upper_button = n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/right_upper_button/state", 1);
  left_shoulder_button =
      n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_shoulder_button/state", 1);
  right_shoulder_button =
      n.advertise<baxter_core_msgs::DigitalIOState>("/robot/digital_io/right_shoulder_button/state", 1);

  QNode::left_arm_nav.button_names.push_back("ok");
  QNode::left_arm_nav.button_names.push_back("back");
  QNode::left_arm_nav.button_names.push_back("show");
  QNode::left_arm_nav.buttons.push_back(false);
  QNode::left_arm_nav.buttons.push_back(false);
  QNode::left_arm_nav.buttons.push_back(false);
  QNode::left_arm_nav.light_names.push_back("inner");
  QNode::left_arm_nav.light_names.push_back("outer");
  QNode::left_arm_nav.lights.push_back(false);
  QNode::left_arm_nav.lights.push_back(false);
  QNode::left_arm_nav.wheel = 0;

  QNode::right_arm_nav.button_names.push_back("ok");
  QNode::right_arm_nav.button_names.push_back("back");
  QNode::right_arm_nav.button_names.push_back("show");
  QNode::right_arm_nav.buttons.push_back(false);
  QNode::right_arm_nav.buttons.push_back(false);
  QNode::right_arm_nav.buttons.push_back(false);
  QNode::right_arm_nav.light_names.push_back("inner");
  QNode::right_arm_nav.light_names.push_back("outer");
  QNode::right_arm_nav.lights.push_back(false);
  QNode::right_arm_nav.lights.push_back(false);
  QNode::right_arm_nav.wheel = 0;

  QNode::left_shoulder_nav.button_names.push_back("ok");
  QNode::left_shoulder_nav.button_names.push_back("back");
  QNode::left_shoulder_nav.button_names.push_back("show");
  QNode::left_shoulder_nav.buttons.push_back(false);
  QNode::left_shoulder_nav.buttons.push_back(false);
  QNode::left_shoulder_nav.buttons.push_back(false);
  QNode::left_shoulder_nav.light_names.push_back("inner");
  QNode::left_shoulder_nav.light_names.push_back("outer");
  QNode::left_shoulder_nav.lights.push_back(false);
  QNode::left_shoulder_nav.lights.push_back(false);
  QNode::left_shoulder_nav.wheel = 0;

  QNode::right_shoulder_nav.button_names.push_back("ok");
  QNode::right_shoulder_nav.button_names.push_back("back");
  QNode::right_shoulder_nav.button_names.push_back("show");
  QNode::right_shoulder_nav.buttons.push_back(false);
  QNode::right_shoulder_nav.buttons.push_back(false);
  QNode::right_shoulder_nav.buttons.push_back(false);
  QNode::right_shoulder_nav.light_names.push_back("inner");
  QNode::right_shoulder_nav.light_names.push_back("outer");
  QNode::right_shoulder_nav.lights.push_back(false);
  QNode::right_shoulder_nav.lights.push_back(false);
  QNode::right_shoulder_nav.wheel = 0;

  QNode::left_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::left_cuff_squeeze.isInputOnly = true;
  QNode::right_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_cuff_squeeze.isInputOnly = true;
  QNode::left_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::left_cuff_ok.isInputOnly = true;
  QNode::right_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_cuff_ok.isInputOnly = true;
  QNode::left_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::left_cuff_grasp.isInputOnly = true;
  QNode::right_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_cuff_grasp.isInputOnly = true;
  QNode::left_shoulder.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::left_shoulder.isInputOnly = true;
  QNode::right_shoulder.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_shoulder.isInputOnly = true;

  left_inner_light_sub = n.subscribe(BAXTER_LEFTIL_TOPIC, 100, &QNode::left_inner_light_sub_cb, this);
  left_outer_light_sub = n.subscribe(BAXTER_LEFTOL_TOPIC, 100, &QNode::left_outer_light_sub_cb, this);
  right_inner_light_sub = n.subscribe(BAXTER_RIGHTIL_TOPIC, 100, &QNode::right_inner_light_sub_cb, this);
  right_outer_light_sub = n.subscribe(BAXTER_RIGHTOL_TOPIC, 100, &QNode::right_outer_light_sub_cb, this);
  torso_left_inner_light_sub = n.subscribe(BAXTER_TORSO_LEFTIL_TOPIC, 100, &QNode::torso_left_inner_light_sub_cb, this);
  torso_left_outer_light_sub = n.subscribe(BAXTER_TORSO_LEFTOL_TOPIC, 100, &QNode::torso_left_outer_light_sub_cb, this);
  torso_right_inner_light_sub =
      n.subscribe(BAXTER_TORSO_RIGHTIL_TOPIC, 100, &QNode::torso_right_inner_light_sub_cb, this);
  torso_right_outer_light_sub =
      n.subscribe(BAXTER_TORSO_RIGHTOL_TOPIC, 100, &QNode::torso_right_outer_light_sub_cb, this);
  start();
  return true;
}

void QNode::left_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::left_arm_nav.lights[0] = bool(msg.state);
}
void QNode::left_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::left_arm_nav.lights[1] = bool(msg.state);
}
void QNode::right_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::right_arm_nav.lights[0] = bool(msg.state);
}
void QNode::right_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::right_arm_nav.lights[1] = bool(msg.state);
}
void QNode::torso_left_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::left_shoulder_nav.lights[0] = bool(msg.state);
}
void QNode::torso_left_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::left_shoulder_nav.lights[1] = bool(msg.state);
}
void QNode::torso_right_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::right_shoulder_nav.lights[0] = bool(msg.state);
}
void QNode::torso_right_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg)
{
  QNode::right_shoulder_nav.lights[1] = bool(msg.state);
}

void QNode::run()
{
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    left_navigator.publish(QNode::left_arm_nav);
    right_navigator.publish(QNode::right_arm_nav);
    torso_left_navigator.publish(QNode::left_shoulder_nav);
    torso_right_navigator.publish(QNode::right_shoulder_nav);

    left_lower_cuff.publish(QNode::left_cuff_squeeze);
    right_lower_cuff.publish(QNode::right_cuff_squeeze);

    left_lower_button.publish(QNode::left_cuff_ok);
    right_lower_button.publish(QNode::right_cuff_ok);

    left_upper_button.publish(QNode::left_cuff_grasp);
    right_upper_button.publish(QNode::right_cuff_grasp);

    left_shoulder_button.publish(QNode::left_shoulder);
    right_shoulder_button.publish(QNode::right_shoulder);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO_NAMED("qnode", "Ros shutdown, proceeding to close the gui.");
}

}  // namespace baxter_sim_io
