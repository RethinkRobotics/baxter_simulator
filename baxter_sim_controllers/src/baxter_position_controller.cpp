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
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <limits> // numeric_limits

namespace baxter_sim_controllers {

bool BaxterPositionController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){
  if(!forward_command_controller::ForwardJointGroupCommandControllerBase<hardware_interface::EffortJointInterface>::init(hw, n)){
    return false;
  }
  else{
    std::string topic_name;
    if( n.getParam("topic", topic_name) ){ // They provided a custom topic to subscribe to
      // Get a node handle that is relative to the base path
      ros::NodeHandle nh_base("~");
      // Create command subscriber custom to baxter
      sub_joint_command_ = nh_base.subscribe<baxter_core_msgs::JointCommand>(topic_name, 1, &BaxterPositionController::jointCommandCB, this);
    }
    else{ // default "command" topic
      // Create command subscriber custom to baxter
      sub_joint_command_ = n.subscribe<baxter_core_msgs::JointCommand>("command", 1, &BaxterPositionController::jointCommandCB, this);
    }
    for (size_t i; i < n_joints_; i++)
    {
      // Add joint name to map (allows unordered list to quickly be mapped to the ordered index)
      joint_to_index_map_.insert(std::pair<std::string,std::size_t>(joint_names_[i],i));
      std::cout<<"joint: "<< joint_names_[i]<<"idx: "<<i<<std::endl;
    }
    return true;
  }
}

void BaxterPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  for(std::size_t i=0; i<n_joints_; i++)
  {
    // Determine if we have a valid position command
    // if it is infinity (not valid), use the current position
    if(commands[i] == std::numeric_limits<double>::infinity()){
      joints_[i].setCommand(this->joints_[i].getPosition());
    }
    else{
      joints_[i].setCommand(commands[i]);
    }
  }
}

void BaxterPositionController::jointCommandCB(const baxter_core_msgs::JointCommandConstPtr& msg){
  // Error check message data
  if( msg->command.size() != msg->names.size() ){
    ROS_ERROR_STREAM_NAMED("jointCommandCB","List of names does not match list of angles size, "
      << msg->command.size() << " != " << msg->names.size() );
    return;
  }
  //Resize the muti array buffer to the number of joints
  //Stuff the vector with infinity to let Update to fill in
  //current joint positions if they are not specified
  std::vector<double> command_multi_array(n_joints_, std::numeric_limits<double>::infinity());
  if(msg->command.size()!=n_joints_){
    ROS_INFO_STREAM_NAMED("jointCommandCB","Dimension of command (" << msg->command.size()
      << ") does not match number of joints (" << n_joints_
      << "). Using current joint positions in place of missing commands.");
  }
  std::map<std::string,std::size_t>::iterator name_it;
  // Map incoming joint names and angles to the correct internal ordering
  for(std::size_t i=0; i<msg->names.size(); i++){
    // Check if the joint name is in our map
    name_it = joint_to_index_map_.find(msg->names[i]);
    if( name_it != joint_to_index_map_.end() ){
      // Joint is in the vector, so we'll update the joint position
      command_multi_array[name_it->second] = msg->command[i];
    }
    else{
      ROS_WARN_STREAM_NAMED("jointCommandCB","Unkown joint commanded: "
        << msg->names[i] <<". Ignoring the command for this joint.");
    }
  }
  commands_buffer_.writeFromNonRT(command_multi_array);
}

} // namespace
PLUGINLIB_EXPORT_CLASS( baxter_sim_controllers::BaxterPositionController,controller_interface::ControllerBase)
