/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Rethink Robotics
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
 *  \author Ian McMahon
 *  \author Dave Coleman
 *  \desc   Multiple joint position controller for Baxter SDK
 */

#ifndef BAXTER_POSITION_CONTROLLER_H
#define BAXTER_POSITION_CONTROLLER_H

#include <ros/node_handle.h>
#include <forward_command_controller/forward_joint_group_command_controller.h> // used for controlling arm joints
#include <hardware_interface/joint_command_interface.h>
#include <baxter_core_msgs/JointCommand.h> // the input command
#include <map> // joint map

namespace baxter_sim_controllers
{
  class BaxterPositionController: public forward_command_controller::ForwardJointGroupCommandControllerBase<hardware_interface::EffortJointInterface>
  {
  public:
    virtual ~BaxterPositionController() {sub_joint_command_.shutdown();}
    virtual bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  private:
    ros::Subscriber sub_joint_command_;
    std::map<std::string,std::size_t> joint_to_index_map_; // allows incoming messages to be quickly ordered
    void jointCommandCB(const baxter_core_msgs::JointCommandConstPtr& msg);
    void update(const ros::Time& time, const ros::Duration& period);
  };
} // namespace

#endif
