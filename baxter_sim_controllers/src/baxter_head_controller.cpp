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

#include <baxter_sim_controllers/baxter_head_controller.h>
#include <pluginlib/class_list_macros.h>

namespace baxter_sim_controllers
{
BaxterHeadController::BaxterHeadController() : new_command(true), update_counter(0)
{
}

BaxterHeadController::~BaxterHeadController()
{
  head_command_sub.shutdown();
}

bool BaxterHeadController::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh)
{
  // Store nodehandle
  nh_ = nh;

  // Get joint sub-controllers
  XmlRpc::XmlRpcValue xml_struct;
  if (!nh_.getParam("joints", xml_struct))
  {
    ROS_ERROR_NAMED("head", "No 'joints' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Make sure it's a struct
  if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("head", "The 'joints' parameter is not a struct (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }

  // Get number of joints
  n_joints = xml_struct.size();
  ROS_INFO_STREAM_NAMED("head", "Initializing BaxterHeadController with " << n_joints << " joints.");

  head_controllers.resize(n_joints);

  int i = 0;  // track the joint id
  for (XmlRpc::XmlRpcValue::iterator joint_it = xml_struct.begin(); joint_it != xml_struct.end(); ++joint_it)
  {
    // Get joint controller
    if (joint_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR_NAMED("head", "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
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

      head_controllers[i].reset(new effort_controllers::JointPositionController());
      head_controllers[i]->init(robot, joint_nh);

    }  // end of joint-namespaces

    // Add joint name to map (allows unordered list to quickly be mapped to the ordered index)
    joint_to_index_map.insert(std::pair<std::string, std::size_t>(head_controllers[i]->getJointName(), i));

    // increment joint i
    ++i;
  }

  // Get controller topic name that it will subscribe to
  if (nh_.getParam("topic", topic_name))
  {  // They provided a custom topic to subscribe to

    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");

    // Create command subscriber custom to baxter
    head_command_sub =
        nh_base.subscribe<baxter_core_msgs::HeadPanCommand>(topic_name, 1, &BaxterHeadController::commandCB, this);
  }
  else  // default "command" topic
  {
    // Create command subscriber custom to baxter
    head_command_sub =
        nh_.subscribe<baxter_core_msgs::HeadPanCommand>("command", 1, &BaxterHeadController::commandCB, this);
  }

  return true;
}

void BaxterHeadController::starting(const ros::Time& time)
{
  baxter_core_msgs::HeadPanCommand initial_command;

  // Fill in the initial command
  for (int i = 0; i < n_joints; i++)
  {
    initial_command.target = head_controllers[i]->getPosition();
  }
  head_command_buffer.initRT(initial_command);
  new_command = true;
}

void BaxterHeadController::stopping(const ros::Time& time)
{
}

void BaxterHeadController::update(const ros::Time& time, const ros::Duration& period)
{
  // Debug info
  update_counter++;
  if (update_counter % 100 == 0)

    updateCommands();

  // Apply joint commands
  for (size_t i = 0; i < n_joints; i++)
  {
    // Update the individual joint controllers
    head_controllers[i]->update(time, period);
  }
}

void BaxterHeadController::updateCommands()
{
  // Check if we have a new command to publish
  if (!new_command)
    return;

  // Go ahead and assume we have proccessed the current message
  new_command = false;

  // Get latest command
  const baxter_core_msgs::HeadPanCommand& command = *(head_command_buffer.readFromRT());

  head_controllers[0]->setCommand(command.target);
}

void BaxterHeadController::commandCB(const baxter_core_msgs::HeadPanCommandConstPtr& msg)
{
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  head_command_buffer.writeFromNonRT(*msg);

  new_command = true;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(baxter_sim_controllers::BaxterHeadController, controller_interface::ControllerBase)
