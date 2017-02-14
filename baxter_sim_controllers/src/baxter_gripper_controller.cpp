/*********************************************************************
 # Copyright (c) 2014 Kei Okada
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

#include <baxter_sim_controllers/baxter_gripper_controller.h>
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>

namespace baxter_sim_controllers
{
BaxterGripperController::BaxterGripperController() : new_command(true), update_counter(0), mimic_idx_(0), main_idx_(0)
{
}

BaxterGripperController::~BaxterGripperController()
{
  gripper_command_sub.shutdown();
}

bool BaxterGripperController::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh)
{
  // Store nodehandle
  nh_ = nh;

  // Get joint sub-controllers
  XmlRpc::XmlRpcValue xml_struct;
  if (!nh_.getParam("joints", xml_struct))
  {
    ROS_ERROR_NAMED("gripper", "No 'joints' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Make sure it's a struct
  if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("gripper", "The 'joints' parameter is not a struct (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Get number of joints
  n_joints = xml_struct.size();
  ROS_INFO_STREAM_NAMED("gripper", "Initializing BaxterGripperController with " << n_joints << " joints.");

  gripper_controllers.resize(n_joints);
  int i = 0;  // track the joint id
  for (XmlRpc::XmlRpcValue::iterator joint_it = xml_struct.begin(); joint_it != xml_struct.end(); ++joint_it)
  {
    // Get joint controller
    if (joint_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR_NAMED("gripper", "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
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

      gripper_controllers[i].reset(new effort_controllers::JointPositionController());
      gripper_controllers[i]->init(robot, joint_nh);

    }  // end of joint-namespaces

    // Set mimic indices
    if (gripper_controllers[i]->joint_urdf_->mimic)
    {
      mimic_idx_ = i;
    }
    else
    {
      main_idx_ = i;
    }

    // Add joint name to map (allows unordered list to quickly be mapped to the ordered index)
    joint_to_index_map.insert(std::pair<std::string, std::size_t>(gripper_controllers[i]->getJointName(), i));

    // increment joint i
    ++i;
  }

  // Get controller topic name that it will subscribe to
  if (nh_.getParam("topic", topic_name))
  {  // They provided a custom topic to subscribe to

    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");

    // Create command subscriber custom to baxter
    gripper_command_sub = nh_base.subscribe<baxter_core_msgs::EndEffectorCommand>(
        topic_name, 1, &BaxterGripperController::commandCB, this);
  }
  else  // default "command" topic
  {
    // Create command subscriber custom to baxter
    gripper_command_sub =
        nh_.subscribe<baxter_core_msgs::EndEffectorCommand>("command", 1, &BaxterGripperController::commandCB, this);
  }
  return true;
}

void BaxterGripperController::starting(const ros::Time& time)
{
}

void BaxterGripperController::stopping(const ros::Time& time)
{
}

void BaxterGripperController::update(const ros::Time& time, const ros::Duration& period)
{
  // Debug info
  update_counter++;
  // TODO: Change to ROS Param (20 Hz)
  if (update_counter % 5 == 0)
  {
    updateCommands();
  }

  // Apply joint commands
  for (size_t i = 0; i < n_joints; i++)
  {
    // Update the individual joint controllers
    gripper_controllers[i]->update(time, period);
  }
}

void BaxterGripperController::updateCommands()
{
  // Check if we have a new command to publish
  if (!new_command)
    return;

  // Go ahead and assume we have proccessed the current message
  new_command = false;

  // Get latest command
  const baxter_core_msgs::EndEffectorCommand& command = *(gripper_command_buffer.readFromRT());

  ROS_DEBUG_STREAM_NAMED("gripper", "Gripper update commands " << command.command << " " << command.args);
  if (command.command != baxter_core_msgs::EndEffectorCommand::CMD_GO)
    return;

  double cmd_position = gripper_controllers[main_idx_]->getPosition();
#ifndef DEPRECATED_YAML_CPP_VERSION
  YAML::Node args = YAML::Load(command.args);
  if (args["position"])
  {
    cmd_position = args["position"].as<double>();
    // Check Command Limits:
    if (cmd_position < 0.0)
    {
      cmd_position = 0.0;
    }
    else if (cmd_position > 100.0)
    {
      cmd_position = 100.0;
    }
    // cmd = ratio * range
    cmd_position = (cmd_position / 100.0) * (gripper_controllers[main_idx_]->joint_urdf_->limits->upper -
                                             gripper_controllers[main_idx_]->joint_urdf_->limits->lower);
  }
#endif
  // Update the individual joint controllers
  ROS_DEBUG_STREAM_NAMED("gripper", gripper_controllers[main_idx_]->joint_urdf_->name << "->setCommand(" << cmd_position << ")");
  gripper_controllers[main_idx_]->setCommand(cmd_position);
  gripper_controllers[mimic_idx_]->setCommand(gripper_controllers[mimic_idx_]->joint_urdf_->mimic->multiplier *
                                                  cmd_position +
                                              gripper_controllers[mimic_idx_]->joint_urdf_->mimic->offset);
}

void BaxterGripperController::commandCB(const baxter_core_msgs::EndEffectorCommandConstPtr& msg)
{
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  gripper_command_buffer.writeFromNonRT(*msg);

  new_command = true;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(baxter_sim_controllers::BaxterGripperController, controller_interface::ControllerBase)
