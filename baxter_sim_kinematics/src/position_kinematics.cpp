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
 *  \desc   Node to wrap/unwrap the messages and calculate the kinematics for the Simulated Baxter
 */

#include <baxter_sim_kinematics/position_kinematics.h>
#include <signal.h>

namespace kinematics
{
static const std::string ref_frame_id = "base";
static const std::string JOINT_STATES = "/robot/joint_states";
static const std::string ROBOT_STATE = "/robot/state";
const int joint_id = 6;  // Modify this to get the FK of different joints

/**
 * Method to initialize the publishing and subscribing topics, services and to acquire the resources required
 * @return true if succeeds.
 */
bool position_kinematics::init(std::string side)
{
  // Robot would be disabled initially
  is_enabled = false;

  // capture the side we are working on
  m_limbName = side;

  // setup handle for the topics
  std::string node_path = "/ExternalTools/" + m_limbName + "/PositionKinematicsNode";
  ros::NodeHandle handle1(node_path);

  static const std::string LIMB_ENDPOINT = "/robot/limb/" + side + "/endpoint_state";

  // setup the service server for the Inverse Kinematics
  m_ikService = handle1.advertiseService("IKService", &position_kinematics::IKCallback, this);

  // Subscribe and advertise the subscribers and publishers accordingly for the Forward Kinematics
  joint_states_sub =
      handle.subscribe<sensor_msgs::JointState>(JOINT_STATES, 100, &position_kinematics::FKCallback, this);
  end_pointstate_pub = handle.advertise<baxter_core_msgs::EndpointState>(LIMB_ENDPOINT, 100);

  // Subscribe to the robot state
  robot_state_sub =
      handle.subscribe<baxter_core_msgs::AssemblyState>(ROBOT_STATE, 100, &position_kinematics::stateCB, this);

  // Initialize the Parameter server with the root_name and tip_name of the Kinematic Chain based on the side
  if (side == "right")
  {
    if (!handle.getParam("right_tip_name", tip_name))
    {
      ROS_FATAL_NAMED("position_kin", "GenericIK: No tip name for Right arm found on parameter server");
      return false;
    }
    if (!handle.getParam("robot_config/right_config/joint_names", joint_names))
    {
      ROS_FATAL_NAMED("position_kin", "GenericIK: No Joint Names for the Right Arm found on parameter server");
      return false;
    }
  }
  else
  {
    if (!handle.getParam("left_tip_name", tip_name))
    {
      ROS_FATAL_NAMED("position_kin", "GenericIK: No tip name for Right arm found on parameter server");
      return false;
    }
    if (!handle.getParam("robot_config/left_config/joint_names", joint_names))
    {
      ROS_FATAL_NAMED("position_kin", "GenericIK: No Joint Names for the Left Arm found on parameter server");
      return false;
    }
  }
  no_jts = joint_names.size();
  m_kinematicsModel = arm_kinematics::Kinematics::create(tip_name, no_jts);
  return true;
}

/**
 * Callback function that checks and sets the robot enabled flag
 */
void position_kinematics::stateCB(const baxter_core_msgs::AssemblyState msg)
{
  if (msg.enabled)
    is_enabled = true;
  else
    is_enabled = false;
}

/**
 * Callback function for the FK subscriber that retrievs the appropriate FK from the Joint states and publishes
 * to the endpoint_state topic
 */
void position_kinematics::FKCallback(const sensor_msgs::JointState msg)
{
  baxter_core_msgs::EndpointState endpoint;

  sensor_msgs::JointState configuration;

  position_kinematics::FilterJointState(&msg, joint);
  // Copy the current Joint positions and names of the appropriate side to the configuration
  endpoint.pose = position_kinematics::FKCalc(joint).pose;

  // Fill out timestamp for endpoint
  endpoint.header.stamp = msg.header.stamp;

  // Publish the PoseStamp of the end effector
  end_pointstate_pub.publish(endpoint);
}

/**
 * Method to Filter the names and positions of the initialized side from the remaining
 */
void position_kinematics::FilterJointState(const sensor_msgs::JointState* msg, sensor_msgs::JointState& res)
{
  // Resize the result to hold the names and positions of the 7 joints
  res.name.resize(joint_names.size());
  res.position.resize(joint_names.size());
  int i = 0;
  for (size_t ind = 0; ind < msg->name.size(); ind++)
  {
    // Retain the names and positions of the joints of the initialized arm
    if (std::find(joint_names.begin(), joint_names.end(), msg->name[ind]) != joint_names.end())
    {
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
geometry_msgs::PoseStamped position_kinematics::FKCalc(const sensor_msgs::JointState configuration)
{
  bool isV;
  geometry_msgs::PoseStamped fk_result;
  isV = m_kinematicsModel->getPositionFK(ref_frame_id, configuration, fk_result);
  return fk_result;
}

/**
 * Callback function for the IK service that responds with the appropriate joint configuration or error message if not
 * found
 */
bool position_kinematics::IKCallback(baxter_core_msgs::SolvePositionIK::Request& req,
                                     baxter_core_msgs::SolvePositionIK::Response& res)
{
  ros::Rate loop_rate(100);
  sensor_msgs::JointState joint_pose;
  res.joints.resize(req.pose_stamp.size());
  res.isValid.resize(req.pose_stamp.size());
  res.result_type.resize(req.pose_stamp.size());
  for (size_t req_index = 0; req_index < req.pose_stamp.size(); req_index++)
  {
    res.isValid[req_index] = 0;
    int valid_inp = 0;

    if (!req.seed_angles.empty() && req.seed_mode != baxter_core_msgs::SolvePositionIKRequest::SEED_CURRENT)
    {
      res.isValid[req_index] =
          m_kinematicsModel->getPositionIK(req.pose_stamp[req_index], req.seed_angles[req_index], &joint_pose);
      res.joints[req_index].name.resize(joint_pose.name.size());
      res.result_type[req_index] = baxter_core_msgs::SolvePositionIKRequest::SEED_USER;
      valid_inp = 1;
    }

    if ((!res.isValid[req_index]) && req.seed_mode != baxter_core_msgs::SolvePositionIKRequest::SEED_USER)
    {
      res.isValid[req_index] = m_kinematicsModel->getPositionIK(req.pose_stamp[req_index], joint, &joint_pose);
      res.joints[req_index].name.resize(joint_pose.name.size());
      res.result_type[req_index] = baxter_core_msgs::SolvePositionIKRequest::SEED_CURRENT;
      valid_inp = 1;
    }

    if (!valid_inp)
    {
      ROS_ERROR_NAMED("position_kin", "Not a valid request message to the IK service");
      return false;
    }

    if (res.isValid[req_index])
    {
      res.joints[req_index].position.resize(joint_pose.position.size());
      res.joints[req_index].name = joint_pose.name;
      res.joints[req_index].position = joint_pose.position;
    }
    else
      res.result_type[req_index] = baxter_core_msgs::SolvePositionIKResponse::RESULT_INVALID;
  }
  loop_rate.sleep();
}

}  // namespace
/***************************************************************************************************/

//! global pointer to Node
kinematics::position_kinematics::poskin_ptr g_pNode;

//! Helper function for
void quitRequested(int)
{
  if (g_pNode)
  {
    g_pNode->exit();
    g_pNode.reset();
  }

  ros::shutdown();
}

/**
 * Entry point for program. Sets up Node, parses
 * command line arguments, then control loop (calling run() on Node)
 */
int main(int argc, char* argv[])
{
  std::string side = argc > 1 ? argv[1] : "";
  if (side != "left" && side != "right")
  {
    fprintf(stderr, "Usage: %s <left | right>\n", argv[0]);
    return 1;
  }
  ros::init(argc, argv, "baxter_sim_kinematics_" + side, ros::init_options::NoSigintHandler);

  // capture signals and attempt to cleanup Node
  signal(SIGINT, quitRequested);

  g_pNode = kinematics::position_kinematics::create(side);

  // test to see if pointer is valid
  if (g_pNode)
  {
    g_pNode->run();
  }

  // position_kinematics calls ros::shutdown upon exit, just return here
  return 0;
}
