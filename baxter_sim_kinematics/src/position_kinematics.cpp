/*********************************************************************
 # Copyright (c) 2014, Rethink Robotics
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
 *  \desc   Node to wrap/unwrap the messages and calculate the kinematics for the Simulated Baxter
 */

#include <position_kinematics.h>
//#include <typeinfo>
#include <signal.h>
//#include <string>

namespace kinematics {

//static const int no_jts = 7;
static const std::string ref_frame_id = "base";
static const std::string JOINT_STATES = "/robot/joint_states";
static const std::string ROBOT_STATE = "/robot/state";
const int joint_id=6;//Modify this to get the FK of different joints

/**
 * Method to initialize the publishing and subscribing topics, services and to acquire the resources required
 * @return true if succeeds.
 */
bool position_kinematics::init(std::string side) {
  //Robot would be disabled initially
  is_enabled = false;

  // capture the side we are working on
  m_limbName = side;

  //setup handle for the topics
  std::string node_path = "/ExternalTools/" + m_limbName
      + "/PositionKinematicsNode";
  ros::NodeHandle handle1(node_path);

  static const std::string LIMB_ENDPOINT = "/robot/limb/" + side + "/endpoint_state";

  //setup the service server for the Inverse Kinematics
  m_ikService = handle1.advertiseService("IKService",
                                         &position_kinematics::IKCallback,
                                         this);

  //Subscribe and advertise the subscribers and publishers accordingly for the Forward Kinematics
  joint_states_sub = handle.subscribe < sensor_msgs::JointState
      > (JOINT_STATES, 100, &position_kinematics::FKCallback, this);
  end_pointstate_pub = handle.advertise < baxter_core_msgs::EndpointState
      > (LIMB_ENDPOINT, 100);

  //Subscribe to the robot state
  robot_state_sub = handle.subscribe < baxter_core_msgs::AssemblyState
      > (ROBOT_STATE, 100, &position_kinematics::stateCB, this);

  if (!handle.getParam("right_tip_name", right_tip_name)) {
    ROS_FATAL("GenericIK: No tip name for Right arm found on parameter server");
    return false;
  }
  if (!handle.getParam("left_tip_name", left_tip_name)) {
    ROS_FATAL("GenericIK: No tip name for Right arm found on parameter server");
    return false;
  }
  no_jts=0;
  //Initialize the Parameter server with the root_name and tip_name of the Kinematic Chain based on the side
  if (side == "right")
    m_kinematicsModel = arm_kinematics::Kinematics::create(right_tip_name, no_jts);
  else
    m_kinematicsModel = arm_kinematics::Kinematics::create(left_tip_name, no_jts);

  return true;

}

/**
 * Callback function that checks and sets the robot enabled flag
 */
void position_kinematics::stateCB(
    const baxter_core_msgs::AssemblyState msg) {
  if (msg.enabled)
    is_enabled = true;
  else
    is_enabled = false;
}

/**
 * Callback function for the FK subscriber that retrievs the appropriate FK from the Joint states and publishes
 * to the endpoint_state topic
 */
void position_kinematics::FKCallback(const sensor_msgs::JointState msg) {
  baxter_core_msgs::EndpointState endpoint;

  arm_kinematics::FKReply reply;
  sensor_msgs::JointState configuration;

  position_kinematics::FilterJointState(&msg, joint);
  //Copy the current Joint positions and names of the appropriate side to the configuration
  reply = position_kinematics::FKCalc(joint);

  //The 6th index holds the PoseStamp of the end effector while the other preceeding indices holds that of the preceeding joints
  endpoint.pose = reply.pose[joint_id].pose;
  end_pointstate_pub.publish(endpoint);
}

/**
 * Method to Filter the names and positions of the initialized side from the remaining
 */
void position_kinematics::FilterJointState(
    const sensor_msgs::JointState *msg, sensor_msgs::JointState &res) {
  // Resize the result to hold the names and positions of the 7 joints
  res.name.resize(no_jts);
  res.position.resize(no_jts);
  int i = 0;
  for (size_t ind = 0; ind < msg->name.size(); ind++) {
    // Retain the names and positions of the joints of the initialized arm
    if ((msg->name[ind]).std::string::find(m_limbName) != std::string::npos) {
      res.name[i] = msg->name[ind];
      res.position[i] = msg->position[ind];
      i++;
    }
  }
}

/**
 * Method to pass the desired configuration of the joints and calculate the FK
 * @return calculated FK
 */
arm_kinematics::FKReply position_kinematics::FKCalc(
    const sensor_msgs::JointState configuration) {
  bool isV;
  arm_kinematics::FKReply fk_result;
  isV = m_kinematicsModel->getPositionFK(ref_frame_id, configuration,
                                         fk_result);
  return fk_result;
}

/**
 * Callback function for the IK service that responds with the appropriate joint configuration or error message if not found
 */
bool position_kinematics::IKCallback(
    baxter_core_msgs::SolvePositionIK::Request &req,
    baxter_core_msgs::SolvePositionIK::Response &res) {
  ros::Rate loop_rate(100);
  sensor_msgs::JointState joint_pose;
  res.joints.resize(req.pose_stamp.size());
  res.isValid.resize(req.pose_stamp.size());
  for (size_t req_index = 0; req_index < req.pose_stamp.size(); req_index++) {
    res.isValid[req_index] = m_kinematicsModel->getPositionIK(
        req.pose_stamp[req_index], joint, &joint_pose);
    res.joints[req_index].name.resize(joint_pose.name.size());

    if (res.isValid[req_index]) {
      res.joints[req_index].position.resize(joint_pose.position.size());
      res.joints[req_index].name = joint_pose.name;
      res.joints[req_index].position = joint_pose.position;
    }
  }
  loop_rate.sleep();
}

}  //namespace
/***************************************************************************************************/

//! global pointer to Node
kinematics::position_kinematics::poskin_ptr g_pNode;

//! Helper function for
void quitRequested(int) {
  ROS_INFO("position_kinematics: Terminating program...");
  if (g_pNode) {
    g_pNode->exit();
    g_pNode.reset();
  }
}

/**
 * Entry point for program. Sets up Node, parses
 * command line arguments, then control loop (calling run() on Node)
 */
int main(int argc, char* argv[]) {
  std::string side = argc > 1 ? argv[1] : "";
  if (side != "left" && side != "right") {
    fprintf(stderr, "Usage: %s <left | right>\n", argv[0]);
    return 1;
  }
  ros::init(argc, argv, "baxter_sim_kinematics_" + side);

  //capture signals and attempt to cleanup Node
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  g_pNode = kinematics::position_kinematics::create(side);

  //test to see if pointer is valid
  if (g_pNode) {
    g_pNode->run();
  }

  //position_kinematics calls ros::shutdown upon exit, just return here
  return 0;
}

