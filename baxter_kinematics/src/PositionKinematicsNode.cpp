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

#include <PositionKinematicsNode.h>
#include <typeinfo>
#include <signal.h>
#include <string>

namespace kinematics {

static const std::string left_tip_name = "left_wrist";
static const std::string right_tip_name = "right_wrist";
static const int no_jts = 7;
static const std::string ref_frame_id="base";
static const std::string topic1="/robot/joint_states";
static const std::string topic4="/robot/state";

/**
* Method to initialize the publishing and subscribing topics, services and to acquire the resources required
* @return true if succeeds.
*/
bool kinematics::PositionKinematicsNode::init(std::string side)
{
	//Robot would be disabled initially
	isEnabled=false;

  	// capture the side we are working on
  	m_limbName = side;

  	//setup handle for the topics
  	std::string node_path = "/ExternalTools/" + m_limbName +"/PositionKinematicsNode";
  	ros::NodeHandle handle1(node_path);

  	//setup a private handle
  	ros::NodeHandle handle;

  	static const std::string topic2="/robot/limb/"+side+"/endpoint_state";
  	static const std::string topic3="/robot/limb/"+side+"/gravity_command";

  	//setup the service server for the Inverse Kinematics
  	m_ikService = handle1.advertiseService("IKService",&PositionKinematicsNode::IKCallback,this);

  	//Subscribe and advertise the subscribers and publishers accordingly for the Forward Kinematics
  	joint_states_sub=handle.subscribe<sensor_msgs::JointState>(topic1,100,&PositionKinematicsNode::FKCallback, this);
  	end_pointstate_pub=handle.advertise<baxter_core_msgs::EndpointState>(topic2,100);
	gravity_pub=handle.advertise<baxter_core_msgs::JointCommand>(topic3,100);

	//Subscribe to the robot state
	robot_state_sub=handle.subscribe<baxter_core_msgs::AssemblyState>(topic4,100,&PositionKinematicsNode::stateCB, this);

  	//Initialize the Parameter server with the root_name and tip_name of the Kinematic Chain based on the side
  	handle.setParam("root_name","torso");

  	if (side=="right")
		m_kinematicsModel = arm_kinematics::Kinematics::create(right_tip_name);
  	else
	{
		m_kinematicsModel = arm_kinematics::Kinematics::create(left_tip_name);
		kinematics::PositionKinematicsNode::test++;
	}
//std::cout<<"Side is "<<m_limbName<<" and test is "<<kinematics::PositionKinematicsNode::test<<std::endl;
  return true;

}

/**
 * Callback function that checks and sets the robot enabled flag
 */
void kinematics::PositionKinematicsNode::stateCB(const baxter_core_msgs::AssemblyState msg)
{
	if (msg.enabled)
		isEnabled=true;
	else
		isEnabled=false;
}

/**
* Callback function for the FK subscriber that retrievs the appropriate FK from the Joint states and publishes
* to the endpoint_state topic
*/
void kinematics::PositionKinematicsNode::FKCallback(const sensor_msgs::JointState msg)
{
	ros::Rate loop_rate(100);
	//if (isEnabled)
	//{
	std::cout<<"Why do we get seg1 fault--------------"<<std::endl;

		bool isV;
		std::vector<double> torques;
		baxter_core_msgs::JointCommand grav_comp;
		baxter_core_msgs::EndpointState endpoint;

		arm_kinematics::FKReply reply;
		sensor_msgs::JointState configuration;

		PositionKinematicsNode::FilterJointState(&msg,joint);
		//Copy the current Joint positions and names of the appropriate side to the configuration
			//configuration=joint;

		reply=PositionKinematicsNode::FKCalc(joint);

		//The 6th index holds the PoseStamp of the end effector while the other preceeding indices holds that of the preceeding joints
		endpoint.pose=reply.pose[6].pose;
		end_pointstate_pub.publish(endpoint);
		loop_rate.sleep();
std::cout<<"Why do we get seg fault--------------"<<std::endl;
		//Gravity Compensation
		isV=m_kinematicsModel->getGravityTorques(joint, torques);
		//isV=false;
		if(isV)
		{
			grav_comp.command.resize(torques.size());
			grav_comp.names.resize(joint.name.size());
				//PositionKinematicsNode::left_grav_cmd.resize(torques.size());
				//PositionKinematicsNode::left_grav_name.resize(joint.name.size());
				//PositionKinematicsNode::right_grav_cmd.resize(torques.size());
				//PositionKinematicsNode::right_grav_name.resize(joint.name.size());
			grav_comp.mode=3;
			grav_comp.command=torques;
			grav_comp.names=joint.name;
			gravity_pub.publish(grav_comp);

			if(m_limbName=="left")
			{
				//kinematics::PositionKinematicsNode::test++;
				PositionKinematicsNode::left_grav_cmd=torques;
				PositionKinematicsNode::left_grav_name=joint.name;
			}
			else
			{

				PositionKinematicsNode::right_grav_cmd=torques;
				PositionKinematicsNode::right_grav_name=joint.name;
			}
			//if(PositionKinematicsNode::int_mutex)
			//{
			PositionKinematicsNode::int_mutex=false;
			PositionKinematicsNode::mutex=false;
			PositionKinematicsNode::grav_cmd.clear();
			PositionKinematicsNode::grav_name.clear();
			PositionKinematicsNode::grav_cmd.reserve(PositionKinematicsNode::left_grav_cmd.size()+PositionKinematicsNode::right_grav_cmd.size());
			PositionKinematicsNode::grav_cmd.insert(PositionKinematicsNode::grav_cmd.end(),PositionKinematicsNode::left_grav_cmd.begin(),PositionKinematicsNode::left_grav_cmd.end());
			PositionKinematicsNode::grav_cmd.insert(PositionKinematicsNode::grav_cmd.end(),PositionKinematicsNode::right_grav_cmd.begin(),PositionKinematicsNode::right_grav_cmd.end());
			PositionKinematicsNode::grav_name.reserve(PositionKinematicsNode::left_grav_name.size()+PositionKinematicsNode::right_grav_name.size());
			PositionKinematicsNode::grav_name.insert(PositionKinematicsNode::grav_name.end(),PositionKinematicsNode::left_grav_name.begin(),PositionKinematicsNode::left_grav_name.end());
			PositionKinematicsNode::grav_name.insert(PositionKinematicsNode::grav_name.end(),PositionKinematicsNode::right_grav_name.begin(),PositionKinematicsNode::right_grav_name.end());
			PositionKinematicsNode::mutex=true;
			PositionKinematicsNode::int_mutex=true;
			//}
			//std::cout<<"The size1 is "<<PositionKinematicsNode::left_grav_name.size()<<std::endl;
			//std::cout<<"The size2 is "<<PositionKinematicsNode::left_grav_name.size()<<std::endl;
			//std::cout<<"The name is "<<PositionKinematicsNode::grav_name[0]<<" and side is "<<m_limbName<<" left is "<<PositionKinematicsNode::left_grav_name[0]<<" right is "<<PositionKinematicsNode::right_grav_name[0]<<std::endl;
			//std::cout<<"Side is "<<m_limbName<<" and test is "<<kinematics::PositionKinematicsNode::test<<std::endl;
		}
		else
			ROS_ERROR("Gravity compensation was not successful");
	//}

}

/**
* Method to Filter the names and positions of the initialized side from the remaining
*/
void kinematics::PositionKinematicsNode::FilterJointState(const sensor_msgs::JointState *msg,sensor_msgs::JointState &res)
{
	// Resize the result to hold the names and positions of the 7 joints
	res.name.resize(no_jts);
	res.position.resize(no_jts);
	int i=0;
	for(size_t ind=0;ind < msg->name.size();ind++)
	{
		// Retain the names and positions of the joints of the initialized arm
		if ((msg->name[ind]).std::string::find(m_limbName) != std::string::npos)
		{
			res.name[i]=msg->name[ind];
			res.position[i]=msg->position[ind];
			i++;
		}
	}
}

/**
* Method to pass the desired configuration of the joints and calculate the FK
* @return calculated FK
*/
arm_kinematics::FKReply
kinematics::PositionKinematicsNode::FKCalc(const sensor_msgs::JointState configuration)
{
	bool isV;
	arm_kinematics::FKReply fk_result;
    	isV = m_kinematicsModel->getPositionFK(ref_frame_id,configuration,fk_result);
  return fk_result;
}

/**
* Callback function for the IK service that responds with the appropriate joint configuration or error message if not found
*/
bool kinematics::PositionKinematicsNode::IKCallback( baxter_core_msgs::SolvePositionIK::Request &req,baxter_core_msgs::SolvePositionIK::Response &res)
{
	ros::Rate loop_rate(100);
	//if(isEnabled)
	//{
		sensor_msgs::JointState joint_pose;
		res.joints.resize(req.pose_stamp.size());
		res.isValid.resize(req.pose_stamp.size());
		for (size_t req_index = 0; req_index < req.pose_stamp.size(); req_index++)
		{
				res.isValid[req_index] = m_kinematicsModel->getPositionIK(
				req.pose_stamp[req_index],joint,&joint_pose);
				res.joints[req_index].name.resize(joint_pose.name.size());

			if (res.isValid[req_index])
			{
					res.joints[req_index].position.resize(joint_pose.position.size());
				res.joints[req_index].name=joint_pose.name;
				res.joints[req_index].position=joint_pose.position;
			}
		}
	//}
	loop_rate.sleep();
}

}//namespace
/***************************************************************************************************/


//! global pointer to Node
kinematics::PositionKinematicsNode::Ptr g_pNode;

//! Helper function for
void quitRequested (int)
{
  ROS_INFO ("PositionKinematicsNode: Terminating program...");
  if(g_pNode)
  {
    g_pNode->exit();
    g_pNode.reset();
  }
}

/**
 * Entry point for program. Sets up Node, parses
 * command line arguments, then control loop (calling run() on Node)
 */
int main(int argc, char* argv[])
{
  std::cout<<"Int is called +++++++++++"<<std::endl;
  	std::string side = argc > 1 ? argv[1] : "";
  	if(side != "left" && side != "right")
  	{
    		fprintf(stderr, "Usage: %s <left | right>\n", argv[0]);
    	   return 1;
  	}
	ros::init(argc,argv,"baxter_kinematics_"+side);
std::cout<<"Ros is initialized 000000000000000000000000000000000000000000000"<<std::endl;
  	//capture signals and attempt to cleanup Node
  	signal(SIGTERM, quitRequested);
  	signal(SIGINT,  quitRequested);
  	signal(SIGHUP,  quitRequested);

	g_pNode = kinematics::PositionKinematicsNode::create(side);
	std::cout<<"side is 000000000000000000000000000000000000000000000 "<<side<<std::endl;

  	//test to see if pointer is valid
  	if(g_pNode)
  	{
    		g_pNode->run();
  	}

  //PositionKinematicsNode calls ros::shutdown upon exit, just return here
  return 0;
}



