/****************************************************************************
* PositionKinematicsNode.h
* Copyright (c) 2008-2012, Rethink Robotics, Inc.
****************************************************************************/


#ifndef POSITIONKINEMATICSNODE_H_
#define POSITIONKINEMATICSNODE_H_

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
//#include <RosMsgTools/RequestServer.h>

#include <motor_control_msgs/JointPosition.h>
#include <motor_control_msgs/SolvePositionFKRequest.h>
#include <motor_control_msgs/SolvePositionFKReply.h>
#include <motor_control_msgs/SolvePositionIKRequest.h>
#include <motor_control_msgs/SolvePositionIKReply.h>
#include <baxter_core_msgs/SolvePositionIK.h>
//#include <RosMsgTools/RosMsgTools.h>
#include <arm_kinematics.h>
#include <sensor_msgs/JointState.h>
#include <motor_control_msgs/FKreply.h>
#include <motor_control_msgs/kinematics.h>
//#include <ExternalTools/HLRKinematicsModel.h>


namespace kinematics {
/*typedef struct fkr{
std::vector<geometry_msgs::Pose> pose;
std::vector<bool> isValid;
}FKReply;*/
//namespace kinematics {
class PositionKinematicsNode
{
protected:
  PositionKinematicsNode() {};

  bool init(std::string side);

public:

  //! return types of create() and createOnStack()
  typedef boost::shared_ptr<PositionKinematicsNode> Ptr;

  /**
   * Factory method that creates a new instance of PositionKinematicsNode(),
   * calls init and returns initialized non-NULL pointer if init succeeds.
   * Returns an empty pointer when init fails.  Design pattern expects that
   * we always check to see if a pointer is not NULL before assuming we can
   * use it.
   *
   * @return boost::shared_ptr
   */
  static Ptr create(std::string side)
  {
    Ptr pPositionKinematicsNode = Ptr(new PositionKinematicsNode());
    if(pPositionKinematicsNode->init(side))
    {
      return pPositionKinematicsNode;
    }
    return Ptr();
  }

  /**
   * Method that serves as the main execution loop of the Node.  This is called in the main() function to 'run'
   * and only returns after exit or ros::shutdown is called.
   */
  void run()
  {
    ROS_INFO("Node entering Run loop");

    //just do spin here (blocks until shutdown), remove while loop
    ros::spin();

    //we have left the ros spin loop, clean up (if needed) then shutdown
    exit();

    //attempt proper shutdown
    ros::shutdown();
  }



  /**
   * Method that allows signals (from their main function) to trigger any
   * cleanup and manually exit the node's run loop.
   * This is usually triggered by capturing a SIGTERM, etc.
   */
  void exit()
  {
    //Do anything to shut down cleanly
    //Note: Run loop will call shutdown before exiting

    m_ikService.shutdown();
  }


private:
	
  //geometry_msgs::Pose
motor_control_msgs::SolvePositionFKReply
FKCallback(const motor_control_msgs::SolvePositionFKRequest *req);

  //motor_control_msgs::SolvePositionIKReply
bool
IKCallback(
    baxter_core_msgs::SolvePositionIK::Request &req,baxter_core_msgs::SolvePositionIK::Response &res);

  void joint_callback(const sensor_msgs::JointState msg);
void JointStatetoJointPosition(const sensor_msgs::JointState *msg,motor_control_msgs::JointPosition &res);
  /**
  * RSDK ROS service based version of the IK call
  */
 // bool IKCallback(baxter_core_msgs::SolvePositionIK::Request &req,
 //            baxter_core_msgs::SolvePositionIK::Response &res);

  //! Initialization variables
  std::string                                     m_limbName;

  //! Subscriber for external API desired Pose msg, returning joint space angles
  ros::ServiceServer                              m_ikService;
  //Heartland::RosMsgTools::RequestServer::Ptr      m_fkServer;
  //Heartland::RosMsgTools::RequestServer::Ptr      m_ikServer;
  //ExternalTools::HLRKinematicsModel::Ptr          m_kinematicsModel;
  arm_kinematics::Kinematics::Ptr m_kinematicsModel;

  // Subscriber for the Joint state commands
  //ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub;
  ros::Publisher end_pointstate_pub;
  motor_control_msgs::JointPosition joint;
};


}

#endif /* POSITIONKINEMATICSNODE_H_ */
