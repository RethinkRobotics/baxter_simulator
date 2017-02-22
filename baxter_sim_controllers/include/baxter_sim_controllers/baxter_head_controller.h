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

#ifndef BAXTER_HEAD_CONTROLLER_H_
#define BAXTER_HEAD_CONTROLLER_H_
#include <ros/node_handle.h>

#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>

//#include <baxter_core_msgs/JointCommand.h> // the input command
#include <baxter_core_msgs/HeadPanCommand.h>
#include <effort_controllers/joint_position_controller.h>  // used for controlling individual joints

namespace baxter_sim_controllers
{
class BaxterHeadController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  BaxterHeadController();
  ~BaxterHeadController();

  bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void updateCommands();

private:
  ros::NodeHandle nh_;

  /**< Last commanded position. */
  realtime_tools::RealtimeBuffer<baxter_core_msgs::HeadPanCommand> head_command_buffer;

  size_t n_joints;
  std::string topic_name;

  std::map<std::string, std::size_t> joint_to_index_map;  // allows incoming messages to be quickly ordered

  bool new_command;  // true when an unproccessed new command is in the realtime buffer
  size_t update_counter;

  // Command subscriber
  ros::Subscriber head_command_sub;

  /**
   * @brief Callback from a recieved goal from the published topic message
   * @param msg trajectory goal
   */
  void commandCB(const baxter_core_msgs::HeadPanCommandConstPtr& msg);

  // Create an effort-based joint position controller for every joint
  std::vector<boost::shared_ptr<effort_controllers::JointPositionController> > head_controllers;
};

}  // namespace

#endif /* BAXTER_HEAD_CONTROLLER_H_ */
