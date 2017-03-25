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

#ifndef ARM_KINEMATICS_H_
#define ARM_KINEMATICS_H_
#include <cstring>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/SEAJointState.h>
#include <gazebo_msgs/SetLinkProperties.h>
#include <gazebo_msgs/GetLinkProperties.h>
#include <algorithm>

namespace arm_kinematics
{
// Structures to pass the messages
typedef struct INFO
{
  std::vector<std::string> link_names;
  std::vector<std::string> joint_names;
} KinematicSolverInfo;

class Kinematics
{
public:
  Kinematics();

  // Disable gravity on Baxter's arms
  bool init_grav();

  /* Initializes the solvers and the other variables required
   *  @returns true is successful
   */
  bool init(std::string tip_name, int& no_jts);
  typedef boost::shared_ptr<Kinematics> Ptr;
  static Ptr create(std::string tip_name, int& no_jts)
  {
    Ptr parm_kinematics = Ptr(new Kinematics());
    if (parm_kinematics->init(tip_name, no_jts))
    {
      return parm_kinematics;
    }
    return Ptr();
  }

  /* Method to calculate the IK for the required end pose
   *  @returns true if successful
   */
  bool getPositionIK(const geometry_msgs::PoseStamped& pose_stamp, const sensor_msgs::JointState& seed,
                     sensor_msgs::JointState* result);

  /* Method to calculate the FK for the required joint configuration
   *  @returns true if successful
   */
  bool getPositionFK(std::string frame_id, const sensor_msgs::JointState& joint_configuration,
                     geometry_msgs::PoseStamped& res);

  /* Method to calculate the torques required to apply at each of the joints for gravity compensation
   *  @returns true is successful
   */
  // bool getGravityTorques(const sensor_msgs::JointState &joint_configuration,
  // std::vector<double> &torquesOut);
  bool getGravityTorques(const sensor_msgs::JointState joint_configuration,
                         baxter_core_msgs::SEAJointState& left_gravity, baxter_core_msgs::SEAJointState& right_gravity,
                         bool isEnabled);

private:
  ros::NodeHandle nh, nh_private;
  std::string root_name, tip_name, grav_left_name, grav_right_name;
  KDL::JntArray joint_min, joint_max;
  KDL::Chain chain, grav_chain_l, grav_chain_r;
  unsigned int num_joints;

  KDL::ChainFkSolverPos_recursive* fk_solver;
  KDL::ChainIkSolverPos_NR_JL* ik_solver_pos;
  KDL::ChainIkSolverVel_pinv* ik_solver_vel;
  KDL::ChainIdSolver_RNE *gravity_solver_l, *gravity_solver_r;

  ros::ServiceServer ik_service, ik_solver_info_service;
  ros::ServiceServer fk_service, fk_solver_info_service;

  tf::TransformListener tf_listener;
  std::vector<int> indd;
  std::vector<std::string> left_joint, right_joint;
  KinematicSolverInfo info, grav_info_l, grav_info_r;

  /* Method to load all the values from the parameter server
   *  @returns true is successful
   */
  bool loadModel(const std::string xml);

  /* Method to read the URDF model and extract the joints
   *  @returns true is successful
   */
  bool readJoints(urdf::Model& robot_model);

  /* Method to calculate the Joint index of a particular joint from the KDL chain
   *  @returns the index of the joint
   */
  int getJointIndex(const std::string& name);

  /* Method to calculate the KDL segment index of a particular segment from the KDL chain
   *  @returns the index of the segment
   */
  int getKDLSegmentIndex(const std::string& name);
};
}
#endif
