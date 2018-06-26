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
 *  \desc   Multiple joint position controller for Baxter SDK
 */

#include <baxter_sim_controllers/baxter_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace baxter_sim_controllers
{
BaxterPositionController::BaxterPositionController() : new_command_(true), update_counter_(0)
{
}

BaxterPositionController::~BaxterPositionController()
{
  position_command_sub_.shutdown();
}

bool BaxterPositionController::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh)
{
  // Store nodehandle
  nh_ = nh;

  // Get joint sub-controllers
  XmlRpc::XmlRpcValue xml_struct;
  if (!nh_.getParam("joints", xml_struct))
  {
    ROS_ERROR_NAMED("position", "No 'joints' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Make sure it's a struct
  if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("position", "The 'joints' parameter is not a struct (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Get number of joints
  n_joints_ = xml_struct.size();
  ROS_INFO_STREAM_NAMED("position", "Initializing BaxterPositionController with " << n_joints_ << " joints.");

  position_controllers_.resize(n_joints_);

  int i = 0;  // track the joint id
  for (XmlRpc::XmlRpcValue::iterator joint_it = xml_struct.begin(); joint_it != xml_struct.end(); ++joint_it)
  {
    // Get joint controller
    if (joint_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR_NAMED("position", "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
                      nh_.getNamespace().c_str());
      return false;
    }

    // Get joint controller name
    std::string joint_controller_name = joint_it->first;

    // Get the joint-namespace nodehandle
    {
      ros::NodeHandle joint_nh(nh_, "joints/" + joint_controller_name);
      ROS_DEBUG_STREAM_NAMED("init", "Loading sub-controller '" << joint_controller_name
                                                               << "', Namespace: " << joint_nh.getNamespace());

      position_controllers_[i].reset(new effort_controllers::JointPositionController());
      position_controllers_[i]->init(robot, joint_nh);

      // DEBUG
      // position_controllers_[i]->printDebug();

    }  // end of joint-namespaces

    // Add joint name to map (allows unordered list to quickly be mapped to the ordered index)
    joint_to_index_map_.insert(std::pair<std::string, std::size_t>(position_controllers_[i]->getJointName(), i));

    // increment joint i
    ++i;
  }

  // Get controller topic name that it will subscribe to
  if (nh_.getParam("topic", topic_name))  // They provided a custom topic to subscribe to
  {
    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");

    // Create command subscriber custom to baxter
    position_command_sub_ =
        nh_base.subscribe<baxter_core_msgs::JointCommand>(topic_name, 1, &BaxterPositionController::commandCB, this);
  }
  else  // default "command" topic
  {
    // Create command subscriber custom to baxter
    position_command_sub_ =
        nh_.subscribe<baxter_core_msgs::JointCommand>("command", 1, &BaxterPositionController::commandCB, this);
  }

  return true;
}

void BaxterPositionController::starting(const ros::Time& time)
{
  baxter_core_msgs::JointCommand initial_command;

  // Fill in the initial command
  for (int i = 0; i < n_joints_; i++)
  {
    initial_command.names.push_back(position_controllers_[i]->getJointName());
    initial_command.command.push_back(position_controllers_[i]->getPosition());
  }
  position_command_buffer_.initRT(initial_command);
  new_command_ = true;
}

void BaxterPositionController::stopping(const ros::Time& time)
{
}

void BaxterPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // Debug info
  verbose_ = false;
  update_counter_++;
  if (update_counter_ % 100 == 0)
    verbose_ = true;

  updateCommands();

  // Apply joint commands
  for (size_t i = 0; i < n_joints_; i++)
  {
    // Update the individual joint controllers
    position_controllers_[i]->update(time, period);
  }
}

void BaxterPositionController::updateCommands()
{
  // Check if we have a new command to publish
  if (!new_command_)
    return;

  // Go ahead and assume we have proccessed the current message
  new_command_ = false;

  // Get latest command
  const baxter_core_msgs::JointCommand& command = *(position_command_buffer_.readFromRT());

  // Error check message data
  if (command.command.size() != command.names.size())
  {
    ROS_ERROR_STREAM_NAMED("update", "List of names does not match list of angles size, "
                                         << command.command.size() << " != " << command.names.size());
    return;
  }

  std::map<std::string, std::size_t>::iterator name_it;

  // Map incoming joint names and angles to the correct internal ordering
  for (size_t i = 0; i < command.names.size(); i++)
  {
    // Check if the joint name is in our map
    name_it = joint_to_index_map_.find(command.names[i]);

    if (name_it != joint_to_index_map_.end())
    {
      // Joint is in the map, so we'll update the joint position
      position_controllers_[name_it->second]->setCommand(command.command[i]);
    }
  }
}

void BaxterPositionController::commandCB(const baxter_core_msgs::JointCommandConstPtr& msg)
{
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  position_command_buffer_.writeFromNonRT(*msg);

  new_command_ = true;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(baxter_sim_controllers::BaxterPositionController, controller_interface::ControllerBase)
