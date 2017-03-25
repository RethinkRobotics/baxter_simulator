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

/**
 *  \author Dave Coleman
 *  \desc   Multiple joint velocity controller for Baxter SDK
 */

#ifndef BAXTER_VELOCITY_CONTROLLER_H
#define BAXTER_VELOCITY_CONTROLLER_H

#include <ros/node_handle.h>

#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>

#include <baxter_core_msgs/JointCommand.h>  // the input command

#include <effort_controllers/joint_velocity_controller.h>  // used for controlling individual joints

namespace baxter_sim_controllers
{
class BaxterVelocityController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  BaxterVelocityController();
  ~BaxterVelocityController();

  bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void updateCommands();

private:
  ros::NodeHandle nh_;

  // Last commanded velocity
  realtime_tools::RealtimeBuffer<baxter_core_msgs::JointCommand> velocity_command_buffer_;

  size_t n_joints_;
  std::string topic_name;

  std::map<std::string, std::size_t> joint_to_index_map_;  // allows incoming messages to be quickly ordered

  bool verbose_;
  bool new_command_;  // true when an unproccessed new command is in the realtime buffer
  size_t update_counter_;

  // Command subscriber
  ros::Subscriber velocity_command_sub_;

  /**
   * @brief Callback from a recieved goal from the published topic message
   * @param msg trajectory goal
   */
  void commandCB(const baxter_core_msgs::JointCommandConstPtr& msg);

  // Create an effort-based joint velocity controller for every joint
  std::vector<boost::shared_ptr<effort_controllers::JointVelocityController> > velocity_controllers_;
};

}  // namespace

#endif
