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
 *  \desc   ROS node that publishes the states captured from the QT events
 */
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <baxter_sim_io/qnode.hpp>
#include <signal.h>

namespace baxter_sim_io {

QNode::QNode(int argc, char** argv)
    : init_argc(argc),
      init_argv(argv) {
  init();
}

QNode::~QNode() {
  if (ros::isStarted()) {
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

bool QNode::init() {
  ros::init(init_argc, init_argv, "baxter_sim_io");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  left_itb = n.advertise<baxter_core_msgs::ITBState>(
      "/robot/itb/left_itb/state", 1);
  right_itb = n.advertise<baxter_core_msgs::ITBState>(
      "/robot/itb/right_itb/state", 1);
  torso_left_itb = n.advertise<baxter_core_msgs::ITBState>(
      "/robot/itb/torso_left_itb/state", 1);
  torso_right_itb = n.advertise<baxter_core_msgs::ITBState>(
      "/robot/itb/torso_right_itb/state", 1);

  left_lower_cuff = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/left_lower_cuff/state", 1);
  right_lower_cuff = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/right_lower_cuff/state", 1);

  left_lower_button = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/left_lower_button/state", 1);
  right_lower_button = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/right_lower_button/state", 1);

  left_upper_button = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/left_upper_button/state", 1);
  right_upper_button = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/right_upper_button/state", 1);
  left_shoulder_button = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/left_shoulder_button/state", 1);
  right_shoulder_button = n.advertise<baxter_core_msgs::DigitalIOState>(
      "/robot/digital_io/right_shoulder_button/state", 1);
  QNode::left_arm_nav.buttons[0] = false;
  QNode::left_arm_nav.buttons[1] = false;
  QNode::left_arm_nav.buttons[2] = false;
  QNode::right_arm_nav.buttons[0] = false;
  QNode::right_arm_nav.buttons[1] = false;
  QNode::right_arm_nav.buttons[2] = false;
  QNode::left_shoulder_nav.buttons[0] = false;
  QNode::left_shoulder_nav.buttons[1] = false;
  QNode::left_shoulder_nav.buttons[2] = false;
  QNode::right_shoulder_nav.buttons[0] = false;
  QNode::right_shoulder_nav.buttons[1] = false;
  QNode::right_shoulder_nav.buttons[2] = false;
  QNode::left_arm_nav.wheel = 0;
  QNode::right_arm_nav.wheel = 0;
  QNode::left_shoulder_nav.wheel = 0;
  QNode::right_shoulder_nav.wheel = 0;
  QNode::left_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::left_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::left_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::left_shoulder.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  QNode::right_shoulder.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {

    left_itb.publish(QNode::left_arm_nav);
    right_itb.publish(QNode::right_arm_nav);
    torso_left_itb.publish(QNode::left_shoulder_nav);
    torso_right_itb.publish(QNode::right_shoulder_nav);

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
  ROS_INFO("Ros shutdown, proceeding to close the gui.");
}

}  // namespace baxter_sim_io
