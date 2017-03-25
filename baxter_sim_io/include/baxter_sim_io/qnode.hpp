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
#ifndef baxter_sim_io_QNODE_HPP_
#define baxter_sim_io_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <baxter_core_msgs/NavigatorState.h>
#include <baxter_core_msgs/DigitalIOState.h>

namespace baxter_sim_io
{
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();
  static baxter_core_msgs::NavigatorState left_arm_nav, right_arm_nav, left_shoulder_nav, right_shoulder_nav;
  static baxter_core_msgs::DigitalIOState left_cuff_squeeze, right_cuff_squeeze, left_cuff_ok, right_cuff_ok,
      left_cuff_grasp, right_cuff_grasp, left_shoulder, right_shoulder;

  void rosShutdown();

private:
  int init_argc;
  char** init_argv;
  ros::Publisher left_navigator, right_navigator, torso_left_navigator, torso_right_navigator, left_lower_cuff,
      right_lower_cuff, left_lower_button, right_lower_button, left_upper_button, right_upper_button,
      left_shoulder_button, right_shoulder_button;
  ros::Subscriber left_inner_light_sub, left_outer_light_sub, right_inner_light_sub, right_outer_light_sub,
      torso_left_inner_light_sub, torso_left_outer_light_sub, torso_right_inner_light_sub, torso_right_outer_light_sub;
  void left_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
  void left_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
  void right_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
  void right_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
  void torso_left_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
  void torso_left_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
  void torso_right_inner_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
  void torso_right_outer_light_sub_cb(const baxter_core_msgs::DigitalIOState& msg);
};

}  // namespace baxter_sim_io

#endif /* baxter_sim_io_QNODE_HPP_ */
