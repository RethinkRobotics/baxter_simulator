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

#include <baxter_sim_controllers/baxter_position_controller.h>
#include <effort_controllers/joint_group_effort_controller.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

namespace baxter_sim_controllers {

void BaxterPositionController::initCommandSub(ros::NodeHandle &n)
{
    if( n.getParam("topic", topic_name) ) // They provided a custom topic to subscribe to
    {
      // Get a node handle that is relative to the base path
      ros::NodeHandle nh_base("~");
      // Create command subscriber custom to baxter
      sub_command_ = nh_base.subscribe<baxter_core_msgs::JointCommand>(topic_name, 1, &BaxterPositionController::commandCB, this);
    }
    else // default "command" topic
    {
      // Create command subscriber custom to baxter
      sub_command_ = n.subscribe<baxter_core_msgs::JointCommand>("command", 1, &BaxterPositionController::commandCB, this);
  }
}

void BaxterPositionController::commandCB(const baxter_core_msgs::JointCommandConstPtr& msg)
{
    // Error check message data
    if( msg->command.size() != msg->names.size() )
    {
      ROS_ERROR_STREAM_NAMED("commandCB","List of names does not match list of angles size, "
        << msg->command.size() << " != " << msg->names.size() );
      return;
    }
    if(msg->command.size()!=n_joints_)
    {
      ROS_ERROR_STREAM_NAMED("commandCB","Dimension of command (" << msg->command.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    std::vector<std::string>::iterator name_idx;
    std::vector<double> command_multi_array;
    command_multi_array.resize(n_joints_);
    size_t cmd_idx;
    // Map incoming joint names and angles to the correct internal ordering
    for(size_t i=0; i<msg->names.size(); i++)
    {
      // Check if the joint name is in the joint name vector
      name_idx = find(joint_names_.begin(), joint_names_.end(), msg->names[i]);
      cmd_idx = name_idx - joint_names_.begin();

      if( name_idx != joint_names_.end() )
      {
        // Joint is in the vector, so we'll update the joint position
        command_multi_array[cmd_idx] = msg->command[i];
      }
    }
    commands_buffer_.writeFromNonRT(command_multi_array);
}


} // namespace
PLUGINLIB_EXPORT_CLASS( baxter_sim_controllers::BaxterPositionController,controller_interface::ControllerBase)
