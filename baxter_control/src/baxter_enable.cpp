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
std::vector<double> grav_cmd;
std::vector<std::string> grav_name;
///bool mutex=true;
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

static const std::string BAXTER_LEFT_GRAVITY_TOPIC = "/robot/limb/left/gravity_command";
static const std::string BAXTER_RIGHT_GRAVITY_TOPIC = "/robot/limb/right/gravity_command";
static const std::string BAXTER_JOINT_TOPIC = "/robot/joint_states";


static const std::string BAXTER_LEFT_LASER_TOPIC = "/robot/laserscan/left_hand_range/state";
static const std::string BAXTER_RIGHT_LASER_TOPIC = "/robot/laserscan/right_hand_range/state";
static const std::string BAXTER_LEFT_IR_TOPIC = "/robot/range/left_hand_range";
static const std::string BAXTER_RIGHT_IR_TOPIC = "/robot/range/right_hand_range";
static const std::string BAXTER_LEFT_IR_STATE_TOPIC = "/robot/analog_io/left_hand_range/state";
static const std::string BAXTER_RIGHT_IR_STATE_TOPIC = "/robot/analog_io/right_hand_range/state";
static const std::string BAXTER_LEFT_IR_INT_TOPIC = "/robot/analog_io/left_hand_range/value_uint32";
static const std::string BAXTER_RIGHT_IR_INT_TOPIC = "/robot/analog_io/right_hand_range/value_uint32";

static const std::string BAXTER_NAV_LIGHT_TOPIC = "/robot/digital_io/command";
static const std::string BAXTER_LEFTIL_TOPIC = "/robot/digital_io/left_itb_light_inner/state";
static const std::string BAXTER_LEFTOL_TOPIC = "/robot/digital_io/left_itb_light_outer/state";
static const std::string BAXTER_TORSO_LEFTIL_TOPIC = "/robot/digital_io/torso_left_itb_light_inner/state";
static const std::string BAXTER_TORSO_LEFTOL_TOPIC = "/robot/digital_io/torso_left_itb_light_outer/state";
static const std::string BAXTER_rightIL_TOPIC = "/robot/digital_io/right_itb_light_inner/state";
static const std::string BAXTER_rightOL_TOPIC = "/robot/digital_io/right_itb_light_outer/state";
static const std::string BAXTER_TORSO_rightIL_TOPIC = "/robot/digital_io/torso_right_itb_light_inner/state";
static const std::string BAXTER_TORSO_rightOL_TOPIC = "/robot/digital_io/torso_right_itb_light_outer/state";

static const int DELAY = 35; // Timeout for publishing a single RSDK image on start up

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

  baxter_enable::leftIL_nav_light.isInputOnly=false;
  baxter_enable::leftOL_nav_light.isInputOnly=false;
  baxter_enable::torso_leftIL_nav_light.isInputOnly=false;
  baxter_enable::torso_leftOL_nav_light.isInputOnly=false;
  baxter_enable::rightIL_nav_light.isInputOnly=false;
  baxter_enable::rightOL_nav_light.isInputOnly=false;
  baxter_enable::torso_rightIL_nav_light.isInputOnly=false;
  baxter_enable::torso_rightOL_nav_light.isInputOnly=false;

  baxter_enable::leftIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
  baxter_enable::leftOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
  baxter_enable::torso_leftIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
  baxter_enable::torso_leftOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
  baxter_enable::rightIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
  baxter_enable::rightOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
  baxter_enable::torso_rightIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
  baxter_enable::torso_rightOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;

  ros::NodeHandle n;

  // Inititalize the publishers
  assembly_state_pub_ = n.advertise<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC,1);
  left_grip_st_pub_ = n.advertise<baxter_core_msgs::EndEffectorState>(BAXTER_LEFT_GRIPPER_ST,1);
  right_grip_st_pub_ = n.advertise<baxter_core_msgs::EndEffectorState>(BAXTER_RIGHT_GRIPPER_ST,1);
  left_grip_prop_pub_ = n.advertise<baxter_core_msgs::EndEffectorProperties>(BAXTER_LEFT_GRIPPER_PROP,1);
  right_grip_prop_pub_ = n.advertise<baxter_core_msgs::EndEffectorProperties>(BAXTER_RIGHT_GRIPPER_PROP,1);
  left_ir_pub = n.advertise<sensor_msgs::Range>(BAXTER_LEFT_IR_TOPIC,1);
  right_ir_pub = n.advertise<sensor_msgs::Range>(BAXTER_RIGHT_IR_TOPIC,1);
  left_ir_state_pub = n.advertise<baxter_core_msgs::AnalogIOState>(BAXTER_LEFT_IR_STATE_TOPIC,1);
  right_ir_state_pub = n.advertise<baxter_core_msgs::AnalogIOState>(BAXTER_RIGHT_IR_STATE_TOPIC,1);
  left_ir_int_pub = n.advertise<std_msgs::UInt32>(BAXTER_LEFT_IR_INT_TOPIC,1);
  right_ir_int_pub = n.advertise<std_msgs::UInt32>(BAXTER_RIGHT_IR_INT_TOPIC,1);
	
  left_itb_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_LEFTIL_TOPIC,1);
  left_itb_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_LEFTOL_TOPIC,1);
  torso_left_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>
				(BAXTER_TORSO_LEFTIL_TOPIC,1);
  torso_left_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>
				(BAXTER_TORSO_LEFTOL_TOPIC,1);

  right_itb_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_rightIL_TOPIC,1);
  right_itb_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_rightOL_TOPIC,1);
  torso_right_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>
				(BAXTER_TORSO_rightIL_TOPIC,1);
  torso_right_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>
				(BAXTER_TORSO_rightOL_TOPIC,1);

  // Initialize the subscribers
  enable_sub_=n.subscribe(BAXTER_ENABLE_TOPIC,100,&baxter_enable::enable_cb,this);
  stop_sub_=n.subscribe(BAXTER_STOP_TOPIC,100,&baxter_enable::stop_cb,this);
  reset_sub_=n.subscribe(BAXTER_RESET_TOPIC,100,&baxter_enable::reset_cb,this);
 // left_grav=n.subscribe(BAXTER_LEFT_GRAVITY_TOPIC,100,&baxter_enable::left_grav_cb,this);
 // right_grav=n.subscribe(BAXTER_RIGHT_GRAVITY_TOPIC,100,&baxter_enable::right_grav_cb,this);
  right_grav=n.subscribe(BAXTER_JOINT_TOPIC,100,&baxter_enable::update_grav,this);
  left_laser_sub=n.subscribe(BAXTER_LEFT_LASER_TOPIC,100,&baxter_enable::left_laser_cb,this);
  right_laser_sub=n.subscribe(BAXTER_RIGHT_LASER_TOPIC,100,&baxter_enable::right_laser_cb,this);
  nav_light_sub=n.subscribe(BAXTER_NAV_LIGHT_TOPIC,100,&baxter_enable::nav_light_cb, this);


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
  //image_transport::Publisher display_pub_ = it.advertise(BAXTER_DISPLAY_TOPIC, 1);

  // Read OpenCV Mat image and convert it to ROS message
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  try
  {
    cv_ptr->image=cv::imread(img_path,CV_LOAD_IMAGE_UNCHANGED);
    if (cv_ptr->image.data)
    {
      cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
      sleep(DELAY); // Wait for the model to load
     // display_pub_.publish(cv_ptr->toImageMsg());
    }
  }
  catch(std::exception e)
  {
    ROS_WARN("Unable to load the startup picture to display on the display");
  }
std::cout<<"going to publish assembly state&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&77"<<std::endl;
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
	baxter_enable::enable=assembly_state_.enabled;
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
	baxter_enable::enable=false;
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
	baxter_enable::enable=false;
}

/**
 * Method to capture the laser data and pass it as IR data for the left arm
 */
void baxter_enable::left_laser_cb(const sensor_msgs::LaserScan &msg)
{
	left_ir.header=msg.header;
	left_ir.min_range=msg.range_min;
	left_ir.max_range=msg.range_max;
	left_ir.radiation_type=1;
	if(msg.ranges[0]<msg.range_max && msg.ranges[0]>msg.range_min)
		left_ir.range=msg.ranges[0];
	else
		left_ir.range=65.5350036621;
	left_ir_state.timestamp=left_ir.header.stamp;
	left_ir_state.value=left_ir.range/1000;
	left_ir_state.isInputOnly=true;
	left_ir_int.data=left_ir.range;
	//if (baxter_enable::enable)
		//{
		left_ir_pub.publish(left_ir);
		left_ir_state_pub.publish(left_ir_state);
		left_ir_int_pub.publish(left_ir_int);
		left_itb_innerL_pub.publish(leftIL_nav_light);
		left_itb_outerL_pub.publish(rightIL_nav_light);
		torso_left_innerL_pub.publish(torso_leftIL_nav_light);
		torso_left_outerL_pub.publish(torso_rightIL_nav_light);
		//}
}

/**
 * Method to capture the laser data and pass it as IR data for the right arm
 */
void baxter_enable::right_laser_cb(const sensor_msgs::LaserScan &msg)
{
	right_ir.header=msg.header;
	right_ir.min_range=msg.range_min;
	right_ir.max_range=msg.range_max;
	right_ir.radiation_type=1;
	if(msg.ranges[0]<msg.range_max && msg.ranges[0]>msg.range_min)
		right_ir.range=msg.ranges[0];
	else
		right_ir.range=65.5350036621;
	right_ir_state.timestamp=right_ir.header.stamp;
	right_ir_state.value=right_ir.range/1000;
	right_ir_state.isInputOnly=true;
	right_ir_int.data=right_ir.range;
	//if (baxter_enable::enable)
	//	{
		right_ir_pub.publish(right_ir);
		right_ir_state_pub.publish(right_ir_state);
		right_ir_int_pub.publish(right_ir_int);
		right_itb_innerL_pub.publish(rightIL_nav_light);
		right_itb_outerL_pub.publish(rightIL_nav_light);
		torso_right_innerL_pub.publish(torso_rightIL_nav_light);
		torso_right_outerL_pub.publish(torso_rightIL_nav_light);
	//	}
}


void baxter_enable::nav_light_cb(const baxter_core_msgs::DigitalOutputCommand &msg)
{
	if(msg.name== "left_itb_light_inner") {
		if (msg.value)
			leftIL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			leftIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else if( msg.name=="right_itb_light_inner") {
		if (msg.value)
			rightIL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			rightIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else if( msg.name=="torso_left_itb_light_inner"){
		if (msg.value)
			torso_leftIL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			torso_leftIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else if( msg.name=="torso_right_itb_light_inner") {
		if (msg.value)
			torso_rightIL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			torso_rightIL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else if( msg.name=="left_itb_light_outer") {
		if (msg.value)
			leftOL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			leftOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else if( msg.name=="right_itb_light_outer") {
		if (msg.value)
			rightOL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			rightOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else if( msg.name=="torso_left_itb_light_outer") {
		if (msg.value)
			torso_leftOL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			torso_leftOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else if( msg.name=="torso_right_itb_light_outer") {
		if (msg.value)
			torso_rightOL_nav_light.state=baxter_core_msgs::DigitalIOState::ON;
		else
			torso_rightOL_nav_light.state=baxter_core_msgs::DigitalIOState::OFF;
	}
	else
		ROS_ERROR("Not a valid componenet id");
	
}
void baxter_enable::left_grav_cb(const baxter_core_msgs::JointCommand &msg)
{
//std::cout<<"left "<<read_l<<std::endl;
	if(!baxter_enable::read_l)
	{
		//std::cout<<"intoread_le"<<std::endl;
	left_grav_cmd.resize(msg.command.size());
	left_grav_name.resize(msg.names.size());
	left_grav_cmd=msg.command;
	left_grav_name=msg.names;
	read_l=true;
		//std::cout<<"last_l"<<std::endl;
	//baxter_enable::update_grav();
	}
}

void baxter_enable::right_grav_cb(const baxter_core_msgs::JointCommand &msg)
{
//std::cout<<"right "<<read_r<<std::endl;
	if(!baxter_enable::read_r)
	{
		//std::cout<<"intoread_r"<<std::endl;
	right_grav_cmd.resize(msg.command.size());
	right_grav_name.resize(msg.names.size());
	right_grav_cmd=msg.command;
	right_grav_name=msg.names;
	read_r=true;
		//std::cout<<"last_r"<<std::endl;
	//baxter_enable::update_grav();
	}
	
}

void baxter_enable::update_grav(const sensor_msgs::JointState msg)
{
  bool isV;
 //isV=m_kinematicsModel.getGravityTorques_n(msg);
	//std::cout<<"update called l and r are "<<baxter_enable::read_l<<" "<<baxter_enable::read_r<<std::endl;
/*	if(baxter_enable::read_l && baxter_enable::read_r)
	{
	    std::cout<<"OOOOOOOOd------------------------------------"<<std::endl;

	    //int samp=0;
	    *mutex =10233;
			//mutex=&samp;
			grav_cmd.clear();
			grav_name.clear();
			grav_cmd.reserve(baxter_enable::left_grav_cmd.size()+baxter_enable::right_grav_cmd.size());
			grav_cmd.insert(grav_cmd.end(),baxter_enable::left_grav_cmd.begin(),baxter_enable::left_grav_cmd.end());
			grav_cmd.insert(grav_cmd.end(),baxter_enable::right_grav_cmd.begin(),baxter_enable::right_grav_cmd.end());
			grav_name.reserve(baxter_enable::left_grav_name.size()+baxter_enable::right_grav_name.size());
			grav_name.insert(grav_name.end(),baxter_enable::left_grav_name.begin(),baxter_enable::left_grav_name.end());
			grav_name.insert(grav_name.end(),baxter_enable::right_grav_name.begin(),baxter_enable::right_grav_name.end());
			//mutex=true;
			baxter_enable::read_l=false;
			baxter_enable::read_r=false;
			std::cout<<"Address is "<<mutex<<" and value is "<<*mutex<<std::endl;
			std::cout<<"the address of ini is "<<&ini<<std::endl;
			//mutex =true;
			//*mutex=samp;
			//std::cout<<"After "<<*mutex<<std::endl;
			//baxter_enable::test=1;
//std::cout<<"for left and right "<<baxter_en::grav_name[0]<<" "<<baxter_en::grav_name[7]<<"-------------------------------------------------------------------------------"<<std::endl;
	}*/
	//if (baxter_enable::test==1)

	//std::cout<<"Ingava"<<std::endl;
	
}
baxter_enable::baxter_enable()
{
 // std::cout<<"her "<<typeid(mutex).name()<<std::endl;
  //int samp;
  mutex=&ini;
  //*mutex=10;
  //std::cout<<"h "<<std::endl;
}
baxter_enable::baxter_enable(int &mut)
{
  mutex=&mut;
  std::cout<<"Address at init is "<<&mut<<mutex<<std::endl;
  std::cout<<"The values at init is "<<mut<<" "<<*mutex<<std::endl;
}
/*baxter_enable::baxter_enable(bool *mut)
{
*mut=*mutex;
}*/


}//namespace

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "baxter_enable");

  std::string img_path = argc > 1 ? argv[1] : "";

  std::cout<<"hh "<<std::endl;
  baxter_en::baxter_enable enable;
  std::cout<<"vvh "<<std::endl;

  bool result=enable.init(img_path);

  return 0;
}
