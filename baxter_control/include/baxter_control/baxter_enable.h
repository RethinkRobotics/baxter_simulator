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
 *  \desc   Node that lies on the top and controls the robot based on the enable, disable, stop and reset  
 *		commands
 */

#ifndef BAXTER_ENABLE_H_
#define BAXTER_ENABLE_H_

#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include "baxter_core_msgs/AssemblyState.h"
#include "baxter_core_msgs/EndEffectorState.h"
#include "baxter_core_msgs/EndEffectorProperties.h"
#include "baxter_core_msgs/JointCommand.h"
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <baxter_core_msgs/AnalogIOState.h>
#include <baxter_core_msgs/DigitalOutputCommand.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <sensor_msgs/JointState.h>
#include <arm_kinematics.h>
//#include <boost/interprocess/shared_memory_object.hpp>
//#include <boost/interprocess/mapped_region.hpp>
//#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/interprocess/managed_shared_memory.hpp> 

namespace baxter_en {
extern std::vector<double> grav_cmd;
extern std::vector<std::string> grav_name;

class baxter_enable {

 public:
  /**
   * Method to initialize the default values for all the variables, instantiate the publishers and 	* subscribers
   * @param img_path that refers the path of the image that loads on start up
   */
baxter_enable(std::vector<std::string> &joint_names, std::vector<double> &grav);
  baxter_enable(){}

  //baxter_enable(int &mut);

  bool init(const std::string &img_path);

  static int test;
  static std::vector<double> left_grav_cmd, right_grav_cmd;
  static std::vector<std::string> left_grav_name, right_grav_name;
  static bool read_l, read_r;
  static bool enable;
  static baxter_core_msgs::DigitalIOState leftIL_nav_light, leftOL_nav_light,
      torso_leftIL_nav_light, torso_leftOL_nav_light, rightIL_nav_light,
      rightOL_nav_light, torso_rightIL_nav_light, torso_rightOL_nav_light;
  static sensor_msgs::JointState JState_msg;
//static int initial;
static int testing, *tessting;
  //static bool mutex;

  //arm_kinematics::Kinematics kin;
 private:
  int *mutex,ini, initial, *in, *i1;
boost::interprocess::managed_shared_memory managed_shm;
  std::vector<std::string>* jn_names;
  std::vector<double>* grav_cmd;
  ros::Subscriber enable_sub_, stop_sub_, reset_sub_, left_grav, right_grav,
      left_laser_sub, right_laser_sub, nav_light_sub;

  ros::Publisher assembly_state_pub_, left_grip_st_pub_, right_grip_st_pub_,
      left_grip_prop_pub_, right_grip_prop_pub_, left_ir_pub, right_ir_pub,
      left_ir_int_pub, right_ir_int_pub, left_ir_state_pub, right_ir_state_pub,
      left_itb_innerL_pub, right_itb_innerL_pub, torso_left_innerL_pub,
      torso_right_innerL_pub, left_itb_outerL_pub, right_itb_outerL_pub,
      torso_left_outerL_pub, torso_right_outerL_pub;

  baxter_core_msgs::AssemblyState assembly_state_;
  baxter_core_msgs::EndEffectorState left_grip_st, right_grip_st;
  baxter_core_msgs::EndEffectorProperties left_grip_prop, right_grip_prop;
  sensor_msgs::Range left_ir, right_ir;
  baxter_core_msgs::AnalogIOState left_ir_state, right_ir_state;
  std_msgs::UInt32 left_ir_int, right_ir_int;
//boost::interprocess::mapped_region region;
  ros::Timer timer_;
  /**
   * Method to publish the loading image on baxter's screen and other publishers that were instantiated
   * @param Nodehandle to initialize the image transport
   * @param img_path that refers the path of the image that loads on start up
   */
  //baxter_enable(bool *mutex);
  void publish(ros::NodeHandle &n, const std::string &img_path);

  /**
   * Callback function to enable the robot
   */
  void enable_cb(const std_msgs::Bool &msg);

  /**
   * Callback function to stop the robot and capture the source of the stop
   */
  void stop_cb(const std_msgs::Empty &msg);

  /**
   * Callback function to reset all the values to False and 0s
   */
  void reset_cb(const std_msgs::Empty &msg);

  void left_laser_cb(const sensor_msgs::LaserScan &msg);

  void right_laser_cb(const sensor_msgs::LaserScan &msg);

  void nav_light_cb(const baxter_core_msgs::DigitalOutputCommand &msg);

  /**
   * Callback function to read the left gravity comp values
   */
  void left_grav_cb(const baxter_core_msgs::JointCommand &msg);

  /**
   * Callback function to read the right gravity comp values
   */
  void right_grav_cb(const baxter_core_msgs::JointCommand &msg);

  /**
   * Method that updates the global gravity variable
   */
  void update_grav(const sensor_msgs::JointState msg);

};

std::vector<double> baxter_enable::left_grav_cmd, baxter_enable::right_grav_cmd;
std::vector<std::string> baxter_enable::left_grav_name,
    baxter_enable::right_grav_name;
bool baxter_enable::read_l = false;
bool baxter_enable::read_r = false;
//bool baxter_enable::mutex=true;
bool baxter_enable::enable = false;
baxter_core_msgs::DigitalIOState baxter_enable::leftIL_nav_light,
    baxter_enable::leftOL_nav_light, baxter_enable::torso_leftIL_nav_light,
    baxter_enable::torso_leftOL_nav_light, baxter_enable::rightIL_nav_light,
    baxter_enable::rightOL_nav_light, baxter_enable::torso_rightIL_nav_light,
    baxter_enable::torso_rightOL_nav_light;
int baxter_enable::test = 0;
sensor_msgs::JointState baxter_enable::JState_msg;
int baxter_enable::testing=0;
//int* baxter_enable::tessting=0;
//baxter_enable::tessting=&baxter_enable::testing;
//int baxter_enable::initial=0;
//arm_kinematics::Kinematics m_kinematicsModel();
}  // namespace

#endif /* BAXTER_ENABLE_H_ */
