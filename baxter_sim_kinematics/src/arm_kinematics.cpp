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
 *  \desc   library that performs the calculations for the IK,FK and Gravity compensation using the KDL library
 */
#include <cstring>
#include <ros/ros.h>
#include <baxter_sim_kinematics/arm_kinematics.h>

namespace arm_kinematics
{
Kinematics::Kinematics() : nh_private("~")
{
}

bool Kinematics::init_grav()
{
  std::string urdf_xml, full_urdf_xml;
  nh.param("urdf_xml", urdf_xml, std::string("robot_description"));
  nh.searchParam(urdf_xml, full_urdf_xml);
  ROS_DEBUG_NAMED("arm_kinematics", "Reading xml file from parameter server");
  std::string result;
  if (!nh.getParam(full_urdf_xml, result))
  {
    ROS_FATAL_NAMED("arm_kinematics", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  if (!nh.getParam("root_name", root_name))
  {
    ROS_FATAL_NAMED("arm_kinematics", "GenericIK: No tip name for gravity found on parameter server");
    return false;
  }
  if (!nh.getParam("grav_right_name", grav_right_name))
  {
    ROS_FATAL_NAMED("arm_kinematics", "GenericIK: No tip name for gravity found on parameter server");
    return false;
  }
  if (!nh.getParam("grav_left_name", grav_left_name))
  {
    ROS_FATAL_NAMED("arm_kinematics", "GenericIK: No tip name for gravity found on parameter server");
    return false;
  }

  // Service client to set the gravity false for the limbs
  ros::ServiceClient get_lp_client = nh.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");

  // Wait for service to become available
  get_lp_client.waitForExistence();

  // Service client to set the gravity false for the limbs
  ros::ServiceClient set_lp_client = nh.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties");

  // Wait for service to become available
  set_lp_client.waitForExistence();

  gazebo_msgs::SetLinkProperties setlinkproperties;
  gazebo_msgs::GetLinkProperties getlinkproperties;

  // Load the right chain and copy them to Right specific variables
  tip_name = grav_right_name;
  if (!loadModel(result))
  {
    ROS_FATAL_NAMED("arm_kinematics", "Could not load models!");
    return false;
  }

  grav_chain_r = chain;
  right_joint.clear();
  right_joint.reserve(chain.getNrOfSegments());
  std::vector<std::string>::iterator idx;
  // Update the right_joint with the fixed joints from the URDF. Get each of the link's properties from
  // GetLinkProperties service and
  // call the SetLinkProperties service with the same set of parameters except for the gravity_mode, which would be
  // disabled. This is
  // to disable the gravity in the links, thereby to eliminate the need for gravity compensation
  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    std::string seg_name = chain.getSegment(i).getName();
    std::string joint_name = chain.getSegment(i).getJoint().getName();
    idx = std::find(info.joint_names.begin(), info.joint_names.end(), joint_name);
    if (idx != info.joint_names.end())
    {
      right_joint.push_back(*idx);
    }
    std::string link_name = chain.getSegment(i).getName();
    getlinkproperties.request.link_name = link_name;
    setlinkproperties.request.link_name = link_name;
    get_lp_client.call(getlinkproperties);
    setlinkproperties.request.com = getlinkproperties.response.com;
    setlinkproperties.request.mass = getlinkproperties.response.mass;
    setlinkproperties.request.ixx = getlinkproperties.response.ixx;
    setlinkproperties.request.iyy = getlinkproperties.response.iyy;
    setlinkproperties.request.izz = getlinkproperties.response.izz;
    setlinkproperties.request.ixy = getlinkproperties.response.ixy;
    setlinkproperties.request.iyz = getlinkproperties.response.iyz;
    setlinkproperties.request.ixz = getlinkproperties.response.ixz;
    setlinkproperties.request.gravity_mode = false;
    set_lp_client.call(setlinkproperties);
  }

  // Create a gravity solver for the right chain
  gravity_solver_r = new KDL::ChainIdSolver_RNE(grav_chain_r, KDL::Vector(0.0, 0.0, -9.8));

  // Load the left chain and copy them to Right specific variable
  tip_name = grav_left_name;
  if (!loadModel(result))
  {
    ROS_FATAL_NAMED("arm_kinematics", "Could not load models!");
    return false;
  }

  grav_chain_l = chain;
  left_joint.clear();
  left_joint.reserve(chain.getNrOfSegments());
  // Update the left_joint with the fixed joints from the URDF. Get each of the link's properties from GetLinkProperties
  // service and
  // call the SetLinkProperties service with the same set of parameters except for the gravity_mode, which would be
  // disabled. This is
  // to disable the gravity in the links, thereby to eliminate the need for gravity compensation
  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    std::string seg_name = chain.getSegment(i).getName();
    std::string joint_name = chain.getSegment(i).getJoint().getName();
    idx = std::find(info.joint_names.begin(), info.joint_names.end(), joint_name);
    if (idx != info.joint_names.end())
    {
      left_joint.push_back(*idx);
    }
    getlinkproperties.request.link_name = chain.getSegment(i).getName();
    std::string link_name = chain.getSegment(i).getName();
    getlinkproperties.request.link_name = link_name;
    setlinkproperties.request.link_name = link_name;
    get_lp_client.call(getlinkproperties);
    setlinkproperties.request.com = getlinkproperties.response.com;
    setlinkproperties.request.mass = getlinkproperties.response.mass;
    setlinkproperties.request.ixx = getlinkproperties.response.ixx;
    setlinkproperties.request.iyy = getlinkproperties.response.iyy;
    setlinkproperties.request.izz = getlinkproperties.response.izz;
    setlinkproperties.request.ixy = getlinkproperties.response.ixy;
    setlinkproperties.request.iyz = getlinkproperties.response.iyz;
    setlinkproperties.request.ixz = getlinkproperties.response.ixz;
    setlinkproperties.request.gravity_mode = false;
    set_lp_client.call(setlinkproperties);
  }

  // Create a gravity solver for the left chain
  gravity_solver_l = new KDL::ChainIdSolver_RNE(grav_chain_l, KDL::Vector(0.0, 0.0, -9.8));
  return true;
}

/* Initializes the solvers and the other variables required
 *  @returns true is successful
 */
bool Kinematics::init(std::string tip, int& no_jts)
{
  // Get URDF XML
  std::string urdf_xml, full_urdf_xml;
  tip_name = tip;
  nh.param("urdf_xml", urdf_xml, std::string("robot_description"));
  nh.searchParam(urdf_xml, full_urdf_xml);
  ROS_DEBUG_NAMED("arm_kinematics", "Reading xml file from parameter server");
  std::string result;
  if (!nh.getParam(full_urdf_xml, result))
  {
    ROS_FATAL_NAMED("arm_kinematics", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  // Get Root and Tip From Parameter Service
  if (!nh.getParam("root_name", root_name))
  {
    ROS_FATAL_NAMED("arm_kinematics", "GenericIK: No root name found on parameter server");
    return false;
  }

  // Load and Read Models
  if (!loadModel(result))
  {
    ROS_FATAL_NAMED("arm_kinematics", "Could not load models!");
    return false;
  }

  // Get Solver Parameters
  int maxIterations;
  double epsilon;
  // KDL::Vector grav;

  nh_private.param("maxIterations", maxIterations, 1000);
  nh_private.param("epsilon", epsilon, 1e-2);

  // Build Solvers
  fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
  ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);
  ik_solver_pos =
      new KDL::ChainIkSolverPos_NR_JL(chain, joint_min, joint_max, *fk_solver, *ik_solver_vel, maxIterations, epsilon);
  no_jts = num_joints;
  return true;
}

/* Method to load all the values from the parameter server
 *  @returns true is successful
 */
bool Kinematics::loadModel(const std::string xml)
{
  urdf::Model robot_model;
  KDL::Tree tree;
  if (!robot_model.initString(xml))
  {
    ROS_FATAL_NAMED("arm_kinematics", "Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree))
  {
    ROS_ERROR_NAMED("arm_kinematics", "Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name, tip_name, chain))
  {
    ROS_ERROR_NAMED("arm_kinematics", "Could not initialize chain object for root_name %s and tip_name %s",
                    root_name.c_str(), tip_name.c_str());
    return false;
  }
  if (!readJoints(robot_model))
  {
    ROS_FATAL_NAMED("arm_kinematics", "Could not read information about the joints");
    return false;
  }

  return true;
}

/* Method to read the URDF model and extract the joints
 *  @returns true is successful
 */
bool Kinematics::readJoints(urdf::Model& robot_model)
{
  num_joints = 0;
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  boost::shared_ptr<const urdf::Joint> joint;
  for (int i = 0; i < chain.getNrOfSegments(); i++)
    while (link && link->name != root_name)
    {
      if (!(link->parent_joint))
      {
        break;
      }
      joint = robot_model.getJoint(link->parent_joint->name);
      if (!joint)
      {
        ROS_ERROR_NAMED("arm_kinematics", "Could not find joint: %s", link->parent_joint->name.c_str());
        return false;
      }
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        ROS_DEBUG_NAMED("arm_kinematics", "adding joint: [%s]", joint->name.c_str());
        num_joints++;
      }
      link = robot_model.getLink(link->getParent()->name);
    }
  joint_min.resize(num_joints);
  joint_max.resize(num_joints);
  info.joint_names.resize(num_joints);
  info.link_names.resize(num_joints);

  link = robot_model.getLink(tip_name);
  unsigned int i = 0;
  while (link && i < num_joints)
  {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      ROS_DEBUG_NAMED("arm_kinematics", "getting bounds for joint: [%s]", joint->name.c_str());

      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        lower = joint->limits->lower;
        upper = joint->limits->upper;
        hasLimits = 1;
      }
      else
      {
        lower = -M_PI;
        upper = M_PI;
        hasLimits = 0;
      }
      int index = num_joints - i - 1;

      joint_min.data[index] = lower;
      joint_max.data[index] = upper;
      info.joint_names[index] = joint->name;
      info.link_names[index] = link->name;
      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  return true;
}

/* Method to calculate the torques required to apply at each of the joints for gravity compensation
 *  @returns true is successful
 */
bool arm_kinematics::Kinematics::getGravityTorques(const sensor_msgs::JointState joint_configuration,
                                                   baxter_core_msgs::SEAJointState& left_gravity,
                                                   baxter_core_msgs::SEAJointState& right_gravity, bool isEnabled)
{
  bool res;
  KDL::JntArray torques_l, torques_r;
  KDL::JntArray jntPosIn_l, jntPosIn_r;
  left_gravity.name = left_joint;
  right_gravity.name = right_joint;
  left_gravity.gravity_model_effort.resize(num_joints);
  right_gravity.gravity_model_effort.resize(num_joints);
  if (isEnabled)
  {
    torques_l.resize(num_joints);
    torques_r.resize(num_joints);
    jntPosIn_l.resize(num_joints);
    jntPosIn_r.resize(num_joints);

    // Copying the positions of the joints relative to its index in the KDL chain
    for (unsigned int j = 0; j < joint_configuration.name.size(); j++)
    {
      for (unsigned int i = 0; i < num_joints; i++)
      {
        if (joint_configuration.name[j] == left_joint.at(i))
        {
          jntPosIn_l(i) = joint_configuration.position[j];
          break;
        }
        else if (joint_configuration.name[j] == right_joint.at(i))
        {
          jntPosIn_r(i) = joint_configuration.position[j];
          break;
        }
      }
    }
    KDL::JntArray jntArrayNull(num_joints);
    KDL::Wrenches wrenchNull_l(grav_chain_l.getNrOfSegments(), KDL::Wrench::Zero());
    int code_l = gravity_solver_l->CartToJnt(jntPosIn_l, jntArrayNull, jntArrayNull, wrenchNull_l, torques_l);
    KDL::Wrenches wrenchNull_r(grav_chain_r.getNrOfSegments(), KDL::Wrench::Zero());
    int code_r = gravity_solver_r->CartToJnt(jntPosIn_r, jntArrayNull, jntArrayNull, wrenchNull_r, torques_r);

    // Check if the gravity was succesfully calculated by both the solvers
    if (code_l >= 0 && code_r >= 0)
    {
      for (unsigned int i = 0; i < num_joints; i++)
      {
        left_gravity.gravity_model_effort[i] = torques_l(i);
        right_gravity.gravity_model_effort[i] = torques_r(i);
      }
      return true;
    }
    else
    {
      ROS_ERROR_THROTTLE_NAMED(1.0, "arm_kinematics", "KT: Failed to compute gravity torques from KDL return code for "
                                                      "left and right arms %d %d",
                               code_l, code_r);
      return false;
    }
  }
  else
  {
    for (unsigned int i = 0; i < num_joints; i++)
    {
      left_gravity.gravity_model_effort[i] = 0;
      right_gravity.gravity_model_effort[i] = 0;
    }
  }
  return true;
}

/* Method to calculate the Joint index of a particular joint from the KDL chain
 *  @returns the index of the joint
 */
int Kinematics::getJointIndex(const std::string& name)
{
  for (unsigned int i = 0; i < info.joint_names.size(); i++)
  {
    if (info.joint_names[i] == name)
      return i;
  }
  return -1;
}

/* Method to calculate the KDL segment index of a particular segment from the KDL chain
 *  @returns the index of the segment
 */
int Kinematics::getKDLSegmentIndex(const std::string& name)
{
  int i = 0;
  while (i < (int)chain.getNrOfSegments())
  {
    if (chain.getSegment(i).getJoint().getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}

/* Method to calculate the IK for the required end pose
 *  @returns true if successful
 */
bool arm_kinematics::Kinematics::getPositionIK(const geometry_msgs::PoseStamped& pose_stamp,
                                               const sensor_msgs::JointState& seed, sensor_msgs::JointState* result)
{
  geometry_msgs::PoseStamped pose_msg_in = pose_stamp;
  tf::Stamped<tf::Pose> transform;
  tf::Stamped<tf::Pose> transform_root;
  tf::poseStampedMsgToTF(pose_msg_in, transform);

  // Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;

  jnt_pos_in.resize(num_joints);
  // Copying the positions of the joints relative to its index in the KDL chain
  for (unsigned int i = 0; i < num_joints; i++)
  {
    int tmp_index = getJointIndex(seed.name[i]);
    if (tmp_index >= 0)
    {
      jnt_pos_in(tmp_index) = seed.position[i];
    }
    else
    {
      ROS_ERROR_NAMED("arm_kinematics", "i: %d, No joint index for %s", i, seed.name[i].c_str());
    }
  }

  // Convert F to our root_frame
  try
  {
    tf_listener.transformPose(root_name, transform, transform_root);
  }
  catch (...)
  {
    ROS_ERROR_NAMED("arm_kinematics", "Could not transform IK pose to frame: %s", root_name.c_str());
    return false;
  }

  KDL::Frame F_dest;
  tf::transformTFToKDL(transform_root, F_dest);

  int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

  if (ik_valid >= 0)
  {
    result->name = info.joint_names;
    result->position.resize(num_joints);
    for (unsigned int i = 0; i < num_joints; i++)
    {
      result->position[i] = jnt_pos_out(i);
      ROS_DEBUG_NAMED("arm_kinematics", "IK Solution: %s %d: %f", result->name[i].c_str(), i, jnt_pos_out(i));
    }
    return true;
  }
  else
  {
    ROS_DEBUG_NAMED("arm_kinematics", "An IK solution could not be found");
    return false;
  }
}

/* Method to calculate the FK for the required joint configuration
 *  @returns true if successful
 */
bool arm_kinematics::Kinematics::getPositionFK(std::string frame_id, const sensor_msgs::JointState& joint_configuration,
                                               geometry_msgs::PoseStamped& result)
{
  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  tf::Stamped<tf::Pose> tf_pose;

  // Copying the positions of the joints relative to its index in the KDL chain
  jnt_pos_in.resize(num_joints);
  for (unsigned int i = 0; i < num_joints; i++)
  {
    int tmp_index = getJointIndex(joint_configuration.name[i]);
    if (tmp_index >= 0)
      jnt_pos_in(tmp_index) = joint_configuration.position[i];
  }

  int num_segments = chain.getNrOfSegments();
  ROS_DEBUG_ONCE_NAMED("arm_kinematics", "Number of Segments in the KDL chain: %d", num_segments);
  if (fk_solver->JntToCart(jnt_pos_in, p_out, num_segments) >= 0)
  {
    tf_pose.frame_id_ = root_name;
    tf_pose.stamp_ = ros::Time();
    tf::poseKDLToTF(p_out, tf_pose);
    try
    {
      tf_listener.transformPose(frame_id, tf_pose, tf_pose);
    }
    catch (...)
    {
      ROS_ERROR_NAMED("arm_kinematics", "Could not transform FK pose to frame: %s", frame_id.c_str());
      return false;
    }
    tf::poseStampedTFToMsg(tf_pose, result);
  }
  else
  {
    ROS_ERROR_NAMED("arm_kinematics", "Could not compute FK for endpoint.");
    return false;
  }
  return true;
}

}  // namespace
