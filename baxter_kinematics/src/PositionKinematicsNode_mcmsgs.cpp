/****************************************************************************
* PositionKinematicsNode.cpp
* Copyright (c) 2008-2012, Rethink Robotics, Inc.
****************************************************************************/

#include <PositionKinematicsNode.h>
#include <typeinfo>
#include <signal.h>
#include <string>

//#include <RosMsgTools/RosMsgTools.h>
//#include <baxter_simulation/arm_kinematics.h>

//namespace rmt = Heartland::RosMsgTools;

//namespace Heartland {
//namespace ExternalTools {
namespace kinematics {

/**
* Method to allocate and initialize all resources required by the IK Node
* @return true if succeeds.
*/
bool PositionKinematicsNode::init(std::string side)
{
  // capture the side we are working on
  m_limbName = side;

  //setup handle for the topics
  std::string node_path = "/ExternalTools/" + m_limbName +
      "/PositionKinematicsNode";
  //ros::NodeHandle handle(node_path);
ros::NodeHandle handle1(node_path);
ros::NodeHandle handle;
  std::string topic1="/robot/joint_states";
  std::string topic2="/robot/limb/"+side+"/endpoint_state";

  //setup the service server
  m_ikService =
      handle1.advertiseService("IKService",&PositionKinematicsNode::IKCallback,this);

  //setup the fk request-reply server
  /*m_fkServer =
    rmt::RequestServer::createServer<
      motor_control_msgs::SolvePositionFKRequest,
      motor_control_msgs::SolvePositionFKReply>
    (node_path + "/FKServer",
     boost::bind(&PositionKinematicsNode::FKCallback, this, _1));*/

  //setup the ik request-reply server
  /*m_ikServer =
    rmt::RequestServer::createServer<
      motor_control_msgs::SolvePositionIKRequest,
      motor_control_msgs::SolvePositionIKReply>
    (node_path + "/IKServer",
     boost::bind(&PositionKinematicsNode::IKCallback, this, _1));*/

//fk_service = nh_private.advertiseService(FK_SERVICE,&Kinematics::getPositionFK,this);

//std::string nss = ros::this_node::getNamespace();
  std::cout<<"The namespace of positionKinematics is ----------------------------------------------"<<std::endl;
  //ROS_DEBUG(nss);
  joint_states_sub=handle.subscribe<sensor_msgs::JointState>(topic1,100,&PositionKinematicsNode::joint_callback, this);
  end_pointstate_pub=handle.advertise<geometry_msgs::Pose>(topic2,100);

 if (side=="right")
{
//handle.setParam("root_name","right_arm_mount");
handle.setParam("root_name","torso");
handle.setParam("tip_name","right_wrist");
}
else
{
//handle.setParam("root_name","left_arm_mount");
handle.setParam("root_name","base");
handle.setParam("tip_name","left_wrist");
}
  // create the kinematics model
/*  m_kinematicsModel =
      ExternalTools::HLRKinematicsModel::create(side);*/
    m_kinematicsModel = arm_kinematics::Kinematics::create();

  return true;

}

void PositionKinematicsNode::joint_callback(const sensor_msgs::JointState msg)
{
       //std::cout<<"Message is successfully passed"<<std::endl;
	motor_control_msgs::SolvePositionFKReply reply;
        motor_control_msgs::SolvePositionFKRequest req;
	//joint=*msg
	PositionKinematicsNode::JointStatetoJointPosition(&msg,joint);
           //   std::cout<<"Message is successfully passed1"<<std::endl;
	req.configuration.resize(1);
        //std::cout<<"The type of req.conf is "<<typeid(req.configuration[0]).name()<<std::endl;
       // std::cout<<"The type of joint is "<<typeid(joint).name()<<std::endl;
        req.configuration[0]=joint;
        //      std::cout<<"Message is successfully passed11111"<<std::endl;
	reply=PositionKinematicsNode::FKCallback(&req);
        //      std::cout<<"Message is successfully passed2"<<std::endl;
	for (size_t reply_index = 0; reply_index < reply.pose.size(); reply_index++)
  	{
		end_pointstate_pub.publish(reply.pose[reply_index]);
	}
	//       std::cout<<"Message is sent"<<std::endl;
}

void PositionKinematicsNode::JointStatetoJointPosition(const sensor_msgs::JointState *msg,motor_control_msgs::JointPosition &res){
       //std::cout<<"Message is successfully passed11"<<std::endl;
       //std::cout<<"the name of limb is "<<m_limbName<<std::endl;
	//motor_control_msgs::JointPosition ress;
        //res.names.resize(msg->name.size());
	res.names.resize(7);
	//res.angles.resize(msg->name.size());
res.angles.resize(7);
int i=0;
	for(size_t ind=0;ind < msg->name.size();ind++)
	{
		//std::cout<<"the res->names before is "<<res->names[ind]<<std::endl;
//std::cout<<"The size is "<<msg->name.size()<<std::endl;
//std::cout<<"The name is "<<msg->name[ind]<<std::endl;
//std::cout<<"The ndfdame is "<<msg->name[ind]<<std::endl;
		if ((msg->name[ind]).std::string::find(m_limbName) != std::string::npos)
		{
//std::cout<<"The ndfdame is "<<std::endl;
		//std::cout<<"the res->names before is "<<res->names[ind]<<std::endl;
		//std::cout<<"the if loop is visited"<<std::endl;
			//ress.names[ind]="Hi";
		//std::cout<<"the ress->names after hi is "<<res.names[ind]<<std::endl;
			res.names[i]=msg->name[ind];
		//std::cout<<"the ress->names after is "<<res.names[ind]<<std::endl;
			res.angles[i]=msg->position[ind];
			i++;
		//std::cout<<"the if loop is about to be exited"<<std::endl;
        // std::cout<<"into if loop"<<std::endl;
		}
     //  std::cout<<"into for loop"<<std::endl;
     //  std::cout<<"into forfr loop"<<std::endl;
	}
}

motor_control_msgs::SolvePositionFKReply
PositionKinematicsNode::FKCallback(
    const motor_control_msgs::SolvePositionFKRequest *req)
{
//std::cout<<"callback is successfull"<<std::endl;
  //std::string frame_id="torso";
std::string frame_id="base";
  motor_control_msgs::SolvePositionFKReply reply;
  reply.pose.resize(req->configuration.size());
  reply.isValid.resize(req->configuration.size());

  for (size_t reply_index = 0; reply_index < reply.pose.size(); reply_index++)
  {
    motor_control_msgs::FKreply fk_result;
    //reply.isValid[reply_index] = m_kinematicsModel->computeFK(frame_id,req.configuration[reply_index], &fk_result);
//std::cout<<"Request is "<<req->configuration[reply_index]<<std::endl;
    reply.isValid[reply_index] = m_kinematicsModel->getPositionFK(frame_id,req->configuration[reply_index], fk_result);
//std::cout<<"Reply is valid "<<reply.isValid[reply_index]<<std::endl;
//std::cout<<"callback1 is successfull"<<std::endl;
    //reply.pose[reply_index] = rmt::toRosPose(fk_result);
    reply.pose[reply_index] = fk_result.pose_stamped[6].pose;
//std::cout<<"fk pose "<<fk_result.pose_stamped[6]<<std::endl;
//std::cout<<"callback 2is successfull"<<std::endl;
  }
//std::cout<<"callback3 is successfull"<<std::endl;
//std::cout<<"Reply is "<<reply<<std::endl;
//std::cout<<"To terminate "<<reply1.pose[0]<<std::endl;
  return reply;
//std::exit(0);
}


//motor_control_msgs::SolvePositionIKReply
/*worked code ---------------------------------------
bool
PositionKinematicsNode::IKCallback(
    motor_control_msgs::kinematics::Request &req,motor_control_msgs::kinematics::Response &res)
{
  //motor_control_msgs::SolvePositionIKReply res;

  // allocate memory for the reponse
std::cout<<"Into the callback"<<std::endl;
  res.joints.resize(req.seeded_pose.size());
  res.isValid.resize(req.seeded_pose.size());

  // iterate over each ik request in the incoming vector
  for (size_t req_index = 0; req_index < req.seeded_pose.size(); req_index++)
  {
    // if the seed is empty use a default seed
    if (req.seeded_pose[req_index].seed.angles.empty() ||
        req.seeded_pose[req_index].seed.names.empty() )
    {
      // compute IK with an assumed seed from the pose map
      res.isValid[req_index] = m_kinematicsModel->getPositionIK(
          req.seeded_pose[req_index].pose_stamp,joint,
          &res.joints[req_index]);

    }
    else
    {
      // compute IK with an assumed seed from the pose map
      res.isValid[req_index] = m_kinematicsModel->getPositionIK(
          req.seeded_pose[req_index].pose_stamp,
          req.seeded_pose[req_index].seed,
          &res.joints[req_index]);
    }
  }

  //return res;
}*/


bool
PositionKinematicsNode::IKCallback(
    baxter_core_msgs::SolvePositionIK::Request &req,baxter_core_msgs::SolvePositionIK::Response &res)
{
  //motor_control_msgs::SolvePositionIKReply res;

  // allocate memory for the reponse
std::cout<<"Into the callback "<<req.pose_stamp.size()<<std::endl;
  res.joints.resize(req.pose_stamp.size());
  res.isValid.resize(req.pose_stamp.size());
motor_control_msgs::JointPosition joint_pose;
  // iterate over each ik request in the incoming vector
  for (size_t req_index = 0; req_index < req.pose_stamp.size(); req_index++)
  {
std::cout<<"Till here successfull"<<std::endl;
    // if the seed is empty use a default seed
    /*if (req.seeded_pose[req_index].seed.angles.empty() ||
        req.seeded_pose[req_index].seed.names.empty() )
    {*/
      // compute IK with an assumed seed from the pose map
      res.isValid[req_index] = m_kinematicsModel->getPositionIK(
          req.pose_stamp[req_index],joint,&joint_pose);
          res.joints[req_index].name.resize(joint_pose.names.size());
std::cout<<"The function was succ "<<res.isValid[req_index]<<std::endl;
std::cout<<"outta function"<<std::endl;

if (res.isValid[req_index])
{
std::cout<<"joint position names "<<joint_pose.names[0]<<std::endl;
std::cout<<"joint position angles "<<joint_pose.angles[0]<<std::endl;
          res.joints[req_index].position.resize(joint_pose.angles.size());
	  res.joints[req_index].name=joint_pose.names;
	  res.joints[req_index].position=joint_pose.angles;
std::cout<<"The names are  "<<res.joints[req_index].name[0]<<std::endl;
std::cout<<"The angles are "<<res.joints[req_index].position[0]<<std::endl;
}
//std::exit(0);

    /*}
    else
    {
      // compute IK with an assumed seed from the pose map
      res.isValid[req_index] = m_kinematicsModel->getPositionIK(
          req.seeded_pose[req_index].pose_stamp,
          req.seeded_pose[req_index].seed,
          &res.joints[req_index]);
    }*/
  }

  //return res;
}


/*
bool
PositionKinematicsNode::IKCallback(baxter_core_msgs::SolvePositionIK::Request  &req,
                                   baxter_core_msgs::SolvePositionIK::Response &res)
{
  // allocate memory for the reponse
  res.joints.resize(req.pose_stamp.size());
  res.isValid.resize(req.pose_stamp.size());

  // iterate over each ik request in the incoming vector
  for (size_t req_index = 0; req_index < req.pose_stamp.size(); req_index++)
  {
    motor_control_msgs::JointPosition joint_pose;

    // compute IK with an assumed seed from the pose map
    res.isValid[req_index] = m_kinematicsModel->computeIK(
        req.pose_stamp[req_index],
        &joint_pose);

    // copy over the data into the baxter_core_msgs RSDK data structure
    res.joints[req_index].name.resize(joint_pose.names.size());
    res.joints[req_index].position.resize(joint_pose.angles.size());
    res.joints[req_index].name = joint_pose.names;
    res.joints[req_index].position = joint_pose.angles;
  }

  return true;
}*/

//}
//}
}


/***************************************************************************************************/


//! global pointer to Node
//Heartland::ExternalTools::PositionKinematicsNode::Ptr g_pNode;
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
  std::string side = argc > 1 ? argv[1] : "";
  if(side != "left" && side != "right")
  {
    fprintf(stderr, "Usage: %s <left | right>\n", argv[0]);
    return 1;
  }
ros::init(argc,argv,"baxter_kinematics");
  //rmt::init(argc, argv, "PositionKinematicsNode_" + side);
  //rmt::connectFrameGraphToTf();

  //capture signals and attempt to cleanup Node
  signal(SIGTERM, quitRequested);
  signal(SIGINT,  quitRequested);
  signal(SIGHUP,  quitRequested);


  //create a Node
  //g_pNode = Heartland::ExternalTools::PositionKinematicsNode::create(side);
g_pNode = kinematics::PositionKinematicsNode::create(side);

  //test to see if pointer is valid
  if(g_pNode)
  {
    g_pNode->run();
  }

  //PositionKinematicsNode calls ros::shutdown upon exit, just return here
  return 0;
}



