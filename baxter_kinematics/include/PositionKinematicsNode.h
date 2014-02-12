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

#ifndef POSITIONKINEMATICSNODE_H_
#define POSITIONKINEMATICSNODE_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/AssemblyState.h>
#include <arm_kinematics.h>
#include <sensor_msgs/JointState.h>

namespace kinematics 
{
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
  
  /**
   * Callback function that checks and sets the robot enabled flag
   */
  void stateCB(const baxter_core_msgs::AssemblyState msg);

  /**
  * Method to pass the desired configuration of the joints and calculate the FK
  * @return calculated FK
  */
  arm_kinematics::FKReply
  FKCalc(const sensor_msgs::JointState req);

  /**
  * Callback function for the IK service that responds with the appropriate joint configuration or error message if not found
  */
  bool IKCallback(baxter_core_msgs::SolvePositionIK::Request &req,baxter_core_msgs::SolvePositionIK::Response &res);

  /**
  * Callback function for the FK subscriber that retrievs the appropriate FK from the Joint states and publishes it to the endpoint   
  * topic
  */
  void FKCallback(const sensor_msgs::JointState msg);

  /**
  * Method to Filter the names and positions of the initialized side from the remaining
  */
  void FilterJointState(const sensor_msgs::JointState *msg, 
					sensor_msgs::JointState &res);

  //! Declaring variables
  bool isEnabled;
  std::string m_limbName;
  ros::ServiceServer m_ikService;
  arm_kinematics::Kinematics::Ptr m_kinematicsModel;
  ros::Subscriber joint_states_sub,robot_state_sub;
  ros::Publisher end_pointstate_pub,gravity_pub;
  sensor_msgs::JointState joint;
};


}

#endif /* POSITIONKINEMATICSNODE_H_ */
