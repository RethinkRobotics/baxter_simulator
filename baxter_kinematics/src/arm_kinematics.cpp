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
 *  \desc   library that performs the calculations for the IK,FK and Gravity compensation using the KDL library
 */
#include <cstring>
#include <ros/ros.h>
#include <arm_kinematics.h>

namespace arm_kinematics {

Kinematics::Kinematics(): nh_private ("~") {
}

bool Kinematics::init_grav()
{
  std::cout<<"Befpre remove"<<std::endl;
  boost::interprocess::shared_memory_object::remove("MySharedMemory");
  std::cout<<"could not delete the memory"<<std::endl;
boost::interprocess::managed_shared_memory shm(boost::interprocess::open_or_create, "MySharedMemory",10000);
std::cout<<"could not open this since it does exit"<<std::endl;

StringAllocator stringallocator(shm.get_segment_manager());

std::pair<MyShmStringVector*,std::size_t> myshmvector = shm.find<MyShmStringVector>("joint_vector");

//std::pair<MyShmStringVector*,std::size_t> myshmvector = shm.find<MyShmStringVector>("myshmvector");
//std::pair<MyShmStringVector*,std::size_t> myshmvector = shm.find<MyShmStringVector>("myshmvector");
std::cout<<"Before the while loop*****************************"<<std::endl;
//boost::interprocess::named_mutex named_mtx(boost::interprocess::open_or_create, "mutex");
//boost::interprocess::interprocess_mutex mutex;
boost::interprocess::named_mutex::remove("mtx");
boost::interprocess::named_mutex mutex(boost::interprocess::open_or_create, "mtx");
while(true)
{
 // boost::interprocess::shared_memory_object::remove("MySharedMemory");
  boost::interprocess::managed_shared_memory shm(boost::interprocess::open_or_create, "MySharedMemory",10000);

  std::pair<MyShmStringVector*,std::size_t> myshmvector = shm.find<MyShmStringVector>("joint_vector");
  //std::cout<<"The vector val is "<<myshmvector.first<<std::endl;
  if (myshmvector.first)
      break;
  }
boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mutex);

std::cout<<"After the while loop _______________________________________________________"<<std::endl;
//named_mtx.lock();

MyShmStringVector* myshmvecto = shm.find_or_construct<MyShmStringVector>("joint_vector")(stringallocator);
//boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(myshmvecto->mutex);
std::cout<<"after the wait&&&&&&&"<<std::endl;
//std::cout<<myshmvecto.first<<std::endl;
std::cout<<myshmvecto->at(3)<<std::endl;
MyShmStringVector* joint_names = shm.find_or_construct<MyShmStringVector>("joint_vector")(stringallocator);


//torquesOut->resize(gravity.size());
 //torquesOut=&gravity;

 std::string urdf_xml, full_urdf_xml;
     //tip_name=tip;
     nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
     nh.searchParam(urdf_xml,full_urdf_xml);
     ROS_DEBUG("Reading xml file from parameter server");
     std::string result;
     if (!nh.getParam(full_urdf_xml, result)) {
         ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
         //return false;
     }

    /* if (!nh.getParam("grav_root_name", root_name)) {
         ROS_FATAL("GenericIK: No root name for gravity found on parameter server");
        // return false;
     }*/
     if (!nh.getParam("grav_left_name", tip_name)) {
         ROS_FATAL("GenericIK: No tip name for gravity found on parameter server");
        // return false;
     }
     if (!nh.getParam("grav_right_name", root_name)) {
         ROS_FATAL("GenericIK: No tip name for gravity found on parameter server");
        // return false;
     }
     root_name="torso";

     tip_name="right_wrist";
     std::cout<<"the root name is "<<root_name<<std::endl;
     std::cout<<"the tip name is "<<tip_name<<std::endl;

     if (!loadModel(result)) {
         ROS_FATAL("Could not load models!");
       //  return false;
     }
     grav_chain_r=chain;
     grav_info_r=info;
     right_joint.clear();

     right_joint.reserve(num_joints);
     for (int i=0;i<info.joint_names.size();i++)
     {
       right_joint.push_back(info.joint_names[i]);
       std::cout<<"Testing at right joint "<<right_joint[i]<<std::endl;
     }
gravity_solver_r = new KDL::ChainIdSolver_RNE(grav_chain_r,KDL::Vector(0.0,0.0,-9.8));
     //tip_name=left_name;
     tip_name="left_wrist";
     std::cout<<"the root1 name is "<<root_name<<std::endl;
     std::cout<<"the tip1 name is "<<tip_name<<std::endl;
     if (!loadModel(result)) {
         ROS_FATAL("Could not load models!");
       //  return false;
     }
     grav_chain_l=chain;
     //grav_info_l=info;
    left_joint.clear();
     left_joint.reserve(num_joints);

     for (int i=0;i<info.joint_names.size();i++)
     {
       left_joint.push_back(info.joint_names[i]);
       std::cout<<"Testing at left joint "<<left_joint[i]<<std::endl;

     }
gravity_solver_l = new KDL::ChainIdSolver_RNE(grav_chain_l,KDL::Vector(0.0,0.0,-9.8));
     //root_name="torso";

     //num_joints=chain.getNrOfJoints();
     indd[joint_names->size()];
     std::cout<<"LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL"<<std::endl;
     std::cout<<"So the joint_names-> size is -----------------------"<<joint_names->size()<<std::endl;
     //sleep(5);
     std::cout<<"The left grav is "<<left_joint[1]<<std::endl;
     std::cout<<"The right grav is "<<right_joint[1]<<std::endl;
    //try{
     for(int k=0;k<joint_names->size();k++)
     {
       std::cout<<" the joints are************************************* "<<joint_names->at(k).c_str()<<std::endl;
       for(int kk=0;kk<num_joints;kk++){
         std::cout<<" the joints are "<<right_joint[kk]<<" "<<left_joint[kk]<<std::endl;
         if(joint_names->at(k).c_str()==left_joint[kk]){
           indd[k]=kk;
           std::cout<<"inside left and so breaking with indd[k] as "<<indd[k]<<std::endl;
           break;
         }
         else if(joint_names->at(k).c_str()==right_joint[kk]){
           indd[k]=kk+num_joints;
           std::cout<<"inside right and so breaking with indd[k] as "<<indd[k]<<std::endl;

           break;
         }
         //else
           //indd[k]=-1;

       }
       std::cout<<"The summary is for k is "<<k<<" the match is at "<<indd[k]<<std::endl;

     }
for (unsigned int j=0; j < joint_names->size(); j++) {
  std::cout<<"The joint names from the def file is ---------"<<joint_names->at(j).c_str()<<std::endl;
     for (unsigned int i=0; i < num_joints; i++) { 
       std::cout<<"Into the second loop whrere num joints and i is"<<num_joints<<" "<<i<<std::endl;
      // std::cout<<"The nexy line is "<<std::endl;
       std::cout<<"The right grav is "<<grav_info_r.joint_names[i]<<std::endl;
       std::cout<<"The left grav is "<<grav_info_l.joint_names[i]<<std::endl;

       //std::cout<<"The left grav is "<<left_joint[i]<<std::endl;

       //std::cout<<"The joint names from the left n right are---------"<<grav_info_l.joint_names[1]<<" "<<grav_info_r.joint_names[1]<<std::endl;
         if (joint_names->at(j).c_str()==grav_info_l.joint_names[i]){
          // std::cout<<"Inside the left side "<<std::endl;
            // indd[j]=i;
            // std::cout<<"So i and indd[j] is "<<i<<" "<<indd[j]<<std::endl;
             break;
         }
	else if (joint_names->at(j).c_str()==grav_info_r.joint_names[i]){
		indd[j]=i+num_joints;
   // std::cout<<"Inside the right side "<<std::endl;
    //  std::cout<<"So i and indd[j] is "<<i<<" "<<indd[j]<<std::endl;
		break;
	}
	else{
		//std::cout<<"Into the other loop "<<std::endl;
		//std::cout<<"For the i "<<i<<" j "<<j<<" indd[j is "<<indd[j]<<std::endl;
	 // indd[j]=-1;
	  //break;
	}
     }
   //  std::cout<<"for j indd[j] is "<<j<<" "<<indd[j]<<std::endl;
     }
     //}

//named_mtx.unlock();

     std::cout<<"Suceeded--------"<<std::endl;
std::cout<<"Num joints is "<<num_joints<<std::endl;
     //gravity_solver = new KDL::ChainIdSolver_RNE(chain,KDL::Vector(0.0,0.0,-9.8));
     std::cout<<"grav_solver init--------"<<std::endl;




 return true;
}

/* Initializes the solvers and the other variables required
*  @returns true is successful
*/
bool Kinematics::init(std::string tip) {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    tip_name=tip;
    nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
    nh.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    // Get Root and Tip From Parameter Service
    if (!nh.getParam("root_name", root_name)) {
        ROS_FATAL("GenericIK: No root name found on parameter server");
        return false;
    }

    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    // Get Solver Parameters
    int maxIterations;
    double epsilon;
    //KDL::Vector grav;

    nh_private.param("maxIterations", maxIterations, 1000);
    nh_private.param("epsilon", epsilon, 1e-2);

    // Build Solvers
    fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
    ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);
    ik_solver_pos = new KDL::ChainIkSolverPos_NR_JL(chain, joint_min, joint_max,
            *fk_solver, *ik_solver_vel, maxIterations, epsilon);
    //gravity_solver = new KDL::ChainIdSolver_RNE(chain,KDL::Vector(0.0,0.0,-9.8));
    return true;
}

/* Method to load all the values from the parameter server
*  @returns true is successful
*/
bool Kinematics::loadModel(const std::string xml) {
    urdf::Model robot_model;
    KDL::Tree tree;
    if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }
    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    if (!tree.getChain(root_name, tip_name, chain)) {
        ROS_ERROR("Could not initialize chain object");
        return false;
    }
    if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        return false;
    }

    return true;
}

/* Method to read the URDF model and extract the joints
*  @returns true is successful
*/
bool Kinematics::readJoints(urdf::Model &robot_model) {
    num_joints = 0;
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;
    for (int i=0;i<chain.getNrOfSegments();i++)
    //std::cout<<"The names are "<<chain.getSegment(i).getJoint().getName()<<std::endl;
    while (link && link->name != root_name) {
      //std::cout<<"Is the problem in getting parent name "<<std::endl;
      if(!(link->parent_joint))
      {
        ROS_ERROR("Finally caught");
        break;
      }
      //std::cout<<"Link name and its parent is "<<link->name<<" "<<link->parent_joint->name<<std::endl;
        joint = robot_model.getJoint(link->parent_joint->name);
        //std::cout<<"Link name and root name are and the joint is "<<link->name<<" "<<root_name<<" "<<joint<<std::endl;
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
            num_joints++;
        }
       // std::cout<<"Outta loop"<<std::endl;
        link = robot_model.getLink(link->getParent()->name);
        //std::cout<<"The link name parent pbm is "<<link->getParent()->name<<std::endl;
       // std::cout<<"Before the next iter "<<std::endl;
       // std::cout<<" at end Link name and its parent is "<<link->name<<" "<<link->parent_joint->name<<std::endl;
    }
//std::cout<<"Outside the loop "<<std::endl;
    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.link_names.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;

            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            info.joint_names[index] = joint->name;
            info.link_names[index] = link->name;
           // std::cout<<"The joint and link names are "<<joint->name<<" "<<link->name<<std::endl;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}

/* Method to calculate the torques required to apply at each of the joints for gravity compensation
*  @returns true is successful
*/
bool arm_kinematics::Kinematics::getGravityTorques(const sensor_msgs::JointState &joint_configuration, std::vector<double> &torquesOut)
{

  KDL::JntArray torques;
  KDL::JntArray jntPosIn;
  int ind [num_joints];

  torques.resize(joint_configuration.position.size());
  jntPosIn.resize(joint_configuration.name.size());
  //torquesOut.resize(joint_configuration.position.size());

  jntPosIn.resize(num_joints);
 // std::cout<<"Get segments "<<chain_g.getNrOfSegments()<<std::endl;
 // std::cout<<"Get joints "<<chain_g.getNrOfJoints()<<std::endl;
 // std::cout<<"Get segments "<<chain.getNrOfSegments()<<std::endl;
 // std::cout<<"Get joints "<<chain.getNrOfJoints()<<std::endl;
  //std::cout<<"Number of joints "<<num_joints<<std::endl;
for (int i=0;i<chain.getNrOfSegments();i++)
 std::cout<<"The names are "<<chain.getSegment(i).getJoint().getName()<<std::endl;
//std::cout<<"Over-------------------"<<std::endl;
  // Copying the positions of the joints relative to its index in the KDL chain
  for (unsigned int i=0; i < num_joints; i++) {
	int tmp_index = getJointIndex(joint_configuration.name[i]);
        if (tmp_index >=0)
		jntPosIn(tmp_index) = joint_configuration.position[i];
	//	ind[i]=tmp_index;
    }

  KDL::JntArray jntArrayNull(joint_configuration.name.size());
  KDL::Wrenches wrenchNull(chain.getNrOfSegments(), KDL::Wrench::Zero());
  int code = gravity_solver_l->CartToJnt(jntPosIn, jntArrayNull, jntArrayNull, wrenchNull, torques);
  std::cout<<"Till here this is called -------------------------"<<std::endl;
  for (int i=0;i<chain.getNrOfSegments();i++)
   std::cout<<"The names are "<<chain.getSegment(i).getJoint().getName()<<std::endl;
  for (int j=0;j<joint_configuration.position.size();j++)
  std::cout<<"The torques are "<<torques(j)<<std::endl;
  for (unsigned int i=0; i < info.joint_names.size(); i++) {
    std::cout<<"The names are "<<info.joint_names[i]<<std::endl;

  }
  if (code >= 0)
  {
    // copy torques into result
    for (unsigned int i = 0; i < joint_configuration.name.size(); i++)
    {
	//torquesOut[ind[i]]=torques(i);
    }
    return true;
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "KT: Failed to compute gravity torques from KDL return code %d", code);
   // torquesOut.clear();
    return false;
  }
}

bool arm_kinematics::Kinematics::getGravityTorques_n(const sensor_msgs::JointState joint_configuration)
{
  boost::interprocess::managed_shared_memory shm(boost::interprocess::open_or_create, "MySharedMemory",10000);
  DoubleAllocator dblallocator (shm.get_segment_manager());
  MyDoubleVector* gravity_cmd = shm.find_or_construct<MyDoubleVector>("grav_vector")(dblallocator);
  boost::interprocess::named_mutex named_mtx(boost::interprocess::open_or_create, "mtx");
  std::cout<<"The problem is in gravity and joint config size is "<<joint_configuration.name.size()<<std::endl;

bool res;
  KDL::JntArray torques_l,torques_r;
  KDL::JntArray jntPosIn_l,jntPosIn_r;
  //int ind [num_joints];

  torques_l.resize(num_joints);
  torques_r.resize(num_joints);
  jntPosIn_l.resize(num_joints);
  jntPosIn_r.resize(num_joints);
  //torquesOut_l->resize(num_joints);
  //torquesOut_r->resize(num_joints);

  //jntPosIn.resize(num_joints);
  //std::cout<<"Get segments "<<chain_g.getNrOfSegments()<<std::endl;
 // std::cout<<"Get joints "<<chain_g.getNrOfJoints()<<std::endl;
//for (int i=0;i<chain_g.getNrOfSegments();i++)
//std::cout<<"The names are "<<chain.getSegment(i).getJoint().getName()<<std::endl;
  // Copying the positions of the joints relative to its index in the KDL chain
  for (unsigned int j=0; j < joint_configuration.name.size(); j++) {
 // int tmp_index = getJointIndex(joint_configuration.name[i]);
     //   if (tmp_index >=0)
    std::cout<<"For joint config name ---------------------"<<joint_configuration.name[j]<<std::endl;
	for (unsigned int i=0; i < num_joints; i++) { 
    std::cout<<"the grav l and grav r are ---------------------"<<left_joint[i]<<" "<<grav_info_r.joint_names[i]<<std::endl;

	 if (joint_configuration.name[j]==left_joint[i]){
jntPosIn_l(i) = joint_configuration.position[j];
   break;
}
else if (joint_configuration.name[j]==grav_info_r.joint_names[i]){
jntPosIn_r(i) = joint_configuration.position[j];
   break;
}
    //jntPosIn(tmp_index) = joint_configuration.position[i];
    //ind[i]=tmp_index;
}
    }
  std::cout<<"Check point 2" <<std::endl;

/*for (unsigned int j=0; j < joint_names.size(); j++) {
     for (unsigned int i=0; i < num_joints; i++) { 
         if (joint_names[j]==grav_info_l.joint_names[i]){
             indd[j]=i;
             break;
         }
	else if (joint_names[j]==grav_info_r.joint_names[i]){
		indd[j]=i+num_joints;
		break;
	}
	else
		indd[j]=-1;
     }
     }*/


  KDL::JntArray jntArrayNull(num_joints);
  KDL::Wrenches wrenchNull_l(grav_chain_l.getNrOfSegments(), KDL::Wrench::Zero());
  int code_l = gravity_solver_l->CartToJnt(jntPosIn_l, jntArrayNull, jntArrayNull, wrenchNull_l, torques_l);
  KDL::Wrenches wrenchNull_r(grav_chain_r.getNrOfSegments(), KDL::Wrench::Zero());
  int code_r = gravity_solver_r->CartToJnt(jntPosIn_r, jntArrayNull, jntArrayNull, wrenchNull_r, torques_r);
  std::cout<<"The size is $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$444"<<gravity_cmd->size()<<std::endl;
  boost::interprocess::named_mutex mutex(boost::interprocess::open_or_create, "mutx");

  if (code_l >= 0 && code_r >= 0)
  {  std::cout<<"Check point 3" <<std::endl;

   // named_mtx.lock();
    // copy torques into result
    std::cout<<"Check point 4" <<std::endl;

    for (unsigned int i = 0; i < gravity_cmd->size(); i++)
    {
      std::cout<<"Check point 5" <<std::endl;

      std::cout<<"The error is for "<<indd[i]<<std::endl;
      if (indd[i]!=-1){
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mutex);
	if (indd[i]<num_joints) {
  //(*torquesOut)[indd[i]]=torques_l(i);
	   std::cout<<"Check point 6 "<<indd[i]<<std::endl;

	  gravity_cmd->at(indd[i])=torques_l(i);
    std::cout<<"The torque and gravity cmd is " <<torques_l(i)<<" "<< gravity_cmd->at(indd[i])<<std::endl;
}
	else {
	   std::cout<<"Check point 2 at ind " <<indd[i]<<std::endl;
  //(*torquesOut)[indd[i]]=torques_r(i);
	   std::cout<<indd[i]<<std::endl;
    gravity_cmd->at(indd[i])=torques_r(i);
    std::cout<<"The torque and gravity cmd is " <<torques_l(i)<<" "<< gravity_cmd->at(indd[i])<<std::endl;

	}
	std::cout<<"Summary:::::::::::::::::::::::::::::::: for index "<<i<<" indd[i] "<<indd[i]<<" the gravity_cmd->at(indd[i]) was "<<gravity_cmd->at(indd[i])<<std::endl;
    }
      //else
      //  gravity_cmd->at(indd[i])=0;

    }
  //  named_mtx.unlock();
    std::cout<<"After the updates ------------------------------------------------------------------------------------"<<gravity_cmd->at(3)<<std::endl;

    return true;
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "KT: Failed to compute gravity torques from KDL return code for left and right arms %d %d", code_l, code_r);
    //torquesOut->clear();
    return false;
  }
}

/* Method to calculate the Joint index of a particular joint from the KDL chain
*  @returns the index of the joint
*/
int Kinematics::getJointIndex(const std::string &name) {
    for (unsigned int i=0; i < info.joint_names.size(); i++) {
        if (info.joint_names[i] == name)
            return i;
    }
    return -1;
}

/* Method to calculate the KDL segment index of a particular segment from the KDL chain
*  @returns the index of the segment
*/
int Kinematics::getKDLSegmentIndex(const std::string &name) {
    int i=0; 
    while (i < (int)chain.getNrOfSegments()) {
	if (chain.getSegment(i).getJoint().getName() == name) {
            return i+1;
        }
        i++;
    }
    return -1;
}

/* Method to calculate the IK for the required end pose
*  @returns true if successful
*/
bool arm_kinematics::Kinematics::getPositionIK(const geometry_msgs::PoseStamped &pose_stamp,
                              const sensor_msgs::JointState &seed,
                              sensor_msgs::JointState *result) {

    geometry_msgs::PoseStamped pose_msg_in = pose_stamp;
    tf::Stamped<tf::Pose> transform;
    tf::Stamped<tf::Pose> transform_root;
    tf::poseStampedMsgToTF( pose_msg_in, transform );

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;

    jnt_pos_in.resize(num_joints);
    // Copying the positions of the joints relative to its index in the KDL chain
    for (unsigned int i=0; i < num_joints; i++) {
	int tmp_index = getJointIndex(seed.name[i]);
        if (tmp_index >=0) {
	    jnt_pos_in(tmp_index) = seed.position[i];
        } else {
            ROS_ERROR("i: %d, No joint index for %s",i,seed.name[i].c_str());
        }
    }

    //Convert F to our root_frame
    try {
        tf_listener.transformPose(root_name, transform, transform_root);
    } catch (...) {
        ROS_ERROR("Could not transform IK pose to frame: %s", root_name.c_str());
       return false;
    }

    KDL::Frame F_dest;
    tf::TransformTFToKDL(transform_root, F_dest);

    int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

    if (ik_valid >= 0) {
        result->name=info.joint_names;
        result->position.resize(num_joints);
        for (unsigned int i=0; i < num_joints; i++) {
            result->position[i] = jnt_pos_out(i);
            ROS_DEBUG("IK Solution: %s %d: %f",result->name[i].c_str(),i,jnt_pos_out(i));
        }
        return true;
    } else {
        ROS_DEBUG("An IK solution could not be found");
        return false;
    }
}

/* Method to calculate the FK for the required joint configuration
*  @returns true if successful
*/
bool arm_kinematics::Kinematics::getPositionFK(std::string frame_id,const sensor_msgs::JointState &joint_configuration,FKReply &result) {
    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    // Copying the positions of the joints relative to its index in the KDL chain
    jnt_pos_in.resize(num_joints);
    for (unsigned int i=0; i < num_joints; i++) {
	int tmp_index = getJointIndex(joint_configuration.name[i]);
        if (tmp_index >=0)
		jnt_pos_in(tmp_index) = joint_configuration.position[i];
    }

    result.pose.resize(joint_configuration.name.size());
    bool valid = true;
      for (unsigned int i=0; i < joint_configuration.name.size(); i++) {
	int segmentIndex = getKDLSegmentIndex(joint_configuration.name[i]);
        ROS_DEBUG("End effector index: %d",segmentIndex);
        ROS_DEBUG("Chain indices: %d",chain.getNrOfSegments());
        if (fk_solver->JntToCart(jnt_pos_in,p_out,segmentIndex) >=0) {
            tf_pose.frame_id_ = root_name;
            tf_pose.stamp_ = ros::Time();
            tf::PoseKDLToTF(p_out,tf_pose);
            try {
                tf_listener.transformPose(frame_id,tf_pose,tf_pose);//resolve frame_id
            } catch (...) {
                ROS_ERROR("Could not transform FK pose to frame: %s",frame_id.c_str());
                return false;
            }
            tf::poseStampedTFToMsg(tf_pose,pose);
	   result.pose[i]=pose;
        } else {
	    ROS_ERROR("Could not compute FK for %s",joint_configuration.name[i].c_str());
            valid = false;
        }
    }
    return true;
}

} //namespace

