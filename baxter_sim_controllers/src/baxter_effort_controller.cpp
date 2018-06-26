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
 *  \desc   Multiple joint effort controller for Baxter SDK
 */

#include <baxter_sim_controllers/baxter_effort_controller.h>
#include <pluginlib/class_list_macros.h>

namespace baxter_sim_controllers
{
BaxterEffortController::BaxterEffortController() : new_command_(true)
// update_counter_(0)
{
}

BaxterEffortController::~BaxterEffortController()
{
  effort_command_sub_.shutdown();
}

bool BaxterEffortController::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh)
{
  // Store nodehandle
  nh_ = nh;

  // Get joint sub-controllers
  XmlRpc::XmlRpcValue xml_struct;
  if (!nh_.getParam("joints", xml_struct))
  {
    ROS_ERROR_NAMED("effort", "No 'joints' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Make sure it's a struct
  if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("effort", "The 'joints' parameter is not a struct (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Get number of joints
  n_joints_ = xml_struct.size();
  ROS_INFO_STREAM_NAMED("effort", "Initializing BaxterEffortController with " << n_joints_ << " joints.");

  effort_controllers_.resize(n_joints_);

  int i = 0;  // track the joint id
  for (XmlRpc::XmlRpcValue::iterator joint_it = xml_struct.begin(); joint_it != xml_struct.end(); ++joint_it)
  {
    // Get joint controller
    if (joint_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR_NAMED("effort", "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
                      nh_.getNamespace().c_str());
      return false;
    }

    // Get joint names from the parameter server
    std::string joint_name[n_joints_];
    // Get joint controller name
    std::string joint_controller_name = joint_it->first;

    // Get the joint-namespace nodehandle
    {
      ros::NodeHandle joint_nh(nh_, "joints/" + joint_controller_name);
      ROS_DEBUG_STREAM_NAMED("init", "Loading sub-controller '" << joint_controller_name
                                                               << "', Namespace: " << joint_nh.getNamespace());

      effort_controllers_[i].reset(new effort_controllers::JointEffortController());
      effort_controllers_[i]->init(robot, joint_nh);

      if (!joint_nh.getParam("joint", joint_name[i]))
      {
        ROS_ERROR_NAMED("effort", "No 'joints' parameter in controller (namespace '%s')",
                        joint_nh.getNamespace().c_str());
        return false;
      }
      // Create a publisher for every joint controller that publishes to the command topic under that controller
      effort_command_pub_[i] = joint_nh.advertise<std_msgs::Float64>("command", 1);

    }  // end of joint-namespaces

    // Add joint name to map (allows unordered list to quickly be mapped to the ordered index)
    joint_to_index_map_.insert(std::pair<std::string, std::size_t>(joint_name[i], i));
    // increment joint i
    ++i;
  }

  // Get controller topic name that it will subscribe to
  if (nh_.getParam("topic", topic_name))  // They provided a custom topic to subscribe to
  {
    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");

    // Create command subscriber custom to baxter
    effort_command_sub_ =
        nh_base.subscribe<baxter_core_msgs::JointCommand>(topic_name, 1, &BaxterEffortController::commandCB, this);
  }
  else  // default "command" topic
  {
    // Create command subscriber custom to baxter
    effort_command_sub_ =
        nh_.subscribe<baxter_core_msgs::JointCommand>("command", 1, &BaxterEffortController::commandCB, this);
  }

  return true;
}

void BaxterEffortController::starting(const ros::Time& time)
{
  for (int i = 0; i < n_joints_; i++)
    effort_controllers_[i]->starting(time);
}

void BaxterEffortController::stopping(const ros::Time& time)
{
}

void BaxterEffortController::update(const ros::Time& time, const ros::Duration& period)
{
  // Check if there are commands to be updated
  if (!new_command_)
    return;
  new_command_ = false;
  for (size_t i = 0; i < n_joints_; i++)
  {
    // Update the individual joint controllers
    effort_controllers_[i]->update(time, period);
  }
}

void BaxterEffortController::commandCB(const baxter_core_msgs::JointCommandConstPtr& msg)
{
  // Check if the number of joints and effort values are equal
  if (msg->command.size() != msg->names.size())
  {
    ROS_ERROR_STREAM_NAMED("update", "List of names does not match list of efforts size, "
                                         << msg->command.size() << " != " << msg->names.size());
    return;
  }

  std::map<std::string, std::size_t>::iterator name_it;
  // Map incoming joint names and effort values to the correct internal ordering
  for (size_t i = 0; i < msg->names.size(); i++)
  {
    // Check if the joint name is in our map
    name_it = joint_to_index_map_.find(msg->names[i]);

    if (name_it != joint_to_index_map_.end())
    {
      // Joint is in the map, so we'll update the joint effort
      m.data = msg->command[i];
      // Publish the joint effort to the corresponding joint controller
      effort_command_pub_[name_it->second].publish(m);
    }
  }
  // Update that new effort values are waiting to be sent to the joints
  new_command_ = true;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(baxter_sim_controllers::BaxterEffortController, controller_interface::ControllerBase)
