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

#include <baxter_control/baxter_enable.h>

namespace baxter_en{

//Topics to subscribe and publish
static const std::string BAXTER_STATE_TOPIC = "/robot/state";
static const std::string BAXTER_ENABLE_TOPIC = "/robot/set_super_enable";
static const std::string BAXTER_STOP_TOPIC = "/robot/set_super_stop";
static const std::string BAXTER_RESET_TOPIC = "/robot/set_super_reset";
static const std::string BAXTER_DISPLAY_TOPIC = "/robot/xdisplay";

static const std::string BAXTER_LEFT_GRIPPER_ST = "/robot/end_effector/left_gripper/state";
static const std::string BAXTER_RIGHT_GRIPPER_ST = "/robot/end_effector/right_gripper/state";
static const std::string BAXTER_LEFT_GRIPPER_PROP = "/robot/end_effector/left_gripper/properties";
static const std::string BAXTER_RIGHT_GRIPPER_PROP = "/robot/end_effector/right_gripper/properties";

static const int TIMEOUT = 25; // Timeout for publishing a single RSDK image on start up

/**
 * Method to initialize the default values for all the variables, instantiate the publishers and subscribers
 * @param img_path that refers the path of the image that loads on start up
 */
bool baxter_enable::init(const std::string &img_path) {

  //Default values for the assembly state
  assembly_state_.enabled = false;             // true if enabled
  assembly_state_.stopped = false;            // true if stopped -- e-stop asserted
  assembly_state_.error = false;              // true if a component of the assembly has an error
  assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;      // button status
  assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;     // If stopped is 										true, the source of the e-stop.

  //Default values for the left and right gripper end effector states
  left_grip_st.timestamp.sec=0;
  left_grip_st.timestamp.nsec=0;
  left_grip_st.id=1;
  left_grip_st.enabled=1;
  left_grip_st.calibrated=1;
  left_grip_st.ready=1;
  left_grip_st.moving=0;
  left_grip_st.gripping=0;
  left_grip_st.missed=0;
  left_grip_st.error=0;
  left_grip_st.position=0.0;
  left_grip_st.force=0.0;
  left_grip_st.state="sample";
  left_grip_st.command="no_op";
  left_grip_st.command_sender="";
  left_grip_st.command_sequence=0;

  right_grip_st=left_grip_st; // Sample values recorded on both the grippers to do the spoof

  //Default values for the left and the right gripper properties
  left_grip_prop.id=65664;
  left_grip_prop.ui_type=2;
  left_grip_prop.manufacturer="test";
  left_grip_prop.product="test";
  left_grip_prop.product="test";
  left_grip_prop.hardware_rev="test";
  left_grip_prop.firmware_rev="test";
  left_grip_prop.firmware_date="test";
  left_grip_prop.controls_grip=true;
  left_grip_prop.senses_grip=true;
  left_grip_prop.reverses_grip=true;
  left_grip_prop.controls_force=true;
  left_grip_prop.senses_force=true;
  left_grip_prop.controls_position=true;
  left_grip_prop.senses_position=true;
  left_grip_prop.properties="";

  right_grip_prop=left_grip_prop; // Sample values recorded on both the grippers to do the spoof

  ros::NodeHandle n;

  // Inititalize the publishers
  assembly_state_pub_ = n.advertise<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC,1);
  left_grip_st_pub_ = n.advertise<baxter_core_msgs::EndEffectorState>(BAXTER_LEFT_GRIPPER_ST,1);
  right_grip_st_pub_ = n.advertise<baxter_core_msgs::EndEffectorState>(BAXTER_RIGHT_GRIPPER_ST,1);
  left_grip_prop_pub_ = n.advertise<baxter_core_msgs::EndEffectorProperties>(BAXTER_LEFT_GRIPPER_PROP,1);
  right_grip_prop_pub_ = n.advertise<baxter_core_msgs::EndEffectorProperties>(BAXTER_RIGHT_GRIPPER_PROP,1);

  // Initialize the subscribers
  enable_sub_=n.subscribe(BAXTER_ENABLE_TOPIC,100,&baxter_enable::enable_cb,this);
  stop_sub_=n.subscribe(BAXTER_STOP_TOPIC,100,&baxter_enable::stop_cb,this);
  reset_sub_=n.subscribe(BAXTER_RESET_TOPIC,100,&baxter_enable::reset_cb,this);

  baxter_enable::publish(n,img_path);

}

/**
 * Method to publish the loading image on baxter's screen and other publishers that were instantiated
 * @param Nodehandle to initialize the image transport
 * @param img_path that refers the path of the image that loads on start up
 */
void baxter_enable::publish(ros::NodeHandle &n,const std::string &img_path) {

  ros::Rate loop_rate(100);
  image_transport::ImageTransport it(n);
  image_transport::Publisher display_pub_ = it.advertise(BAXTER_DISPLAY_TOPIC, 1);

  // Read OpenCV Mat image and convert it to ROS message
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  try
  {
    cv_ptr->image=cv::imread(img_path,CV_LOAD_IMAGE_UNCHANGED);
    if (cv_ptr->image.data)
    {
      cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
      sleep(TIMEOUT); // Wait for the model to load

      display_pub_.publish(cv_ptr->toImageMsg());
    }
  }
  catch(std::exception e)
  {
    ROS_WARN("Unable to load the startup picture to display on the display");
  }

  while (ros::ok())
  {
	assembly_state_pub_.publish(assembly_state_);
	left_grip_st_pub_.publish(left_grip_st);
	right_grip_st_pub_.publish(right_grip_st);
	left_grip_prop_pub_.publish(left_grip_prop);
	right_grip_prop_pub_.publish(right_grip_prop);
      	ros::spinOnce();
      	loop_rate.sleep();
  }

}
/**
 * Method to enable the robot
 */
void baxter_enable::enable_cb(const std_msgs::Bool &msg)
{

	if (msg.data){
	  assembly_state_.enabled=true;
	}

	else {
	  assembly_state_.enabled=false;
	}
	assembly_state_.stopped=false;
	assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
	assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
}

/**
 * Method to stop the robot and capture the source of the stop
 */
void baxter_enable::stop_cb(const std_msgs::Empty &msg)
{
	assembly_state_.enabled=false;
	assembly_state_.stopped=true;
	assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
	assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_BRAIN;
}

/**
 * Method resets all the values to False and 0s
 */
void baxter_enable::reset_cb(const std_msgs::Empty &msg)
{
	assembly_state_.enabled=false;
	assembly_state_.stopped=false;
	assembly_state_.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
	assembly_state_.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
}

}//namespace

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "baxter_enable");

  std::string img_path = argc > 1 ? argv[1] : "";
  
  baxter_en::baxter_enable enable;

  bool result=enable.init(img_path);

  return 0;
}
