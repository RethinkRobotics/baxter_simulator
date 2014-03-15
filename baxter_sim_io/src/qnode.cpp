/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/baxter_sim_io/qnode.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baxter_sim_io {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
	init();
	}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"baxter_sim_io");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	left_itb = n.advertise<baxter_core_msgs::ITBState>
			("/robot/itb/left_itb/state", 1);
	right_itb = n.advertise<baxter_core_msgs::ITBState>
			("/robot/itb/right_itb/state", 1);
	torso_left_itb = n.advertise<baxter_core_msgs::ITBState>
			("/robot/itb/torso_left_itb/state", 1);
	torso_right_itb = n.advertise<baxter_core_msgs::ITBState>
			("/robot/itb/torso_right_itb/state", 1);

	left_lower_cuff = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/left_lower_cuff/state", 1);
	right_lower_cuff = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/right_lower_cuff/state", 1);

	left_lower_button = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/left_lower_button/state", 1);
	right_lower_button = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/right_lower_button/state", 1);

	left_upper_button = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/left_upper_button/state", 1);
	right_upper_button = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/right_upper_button/state", 1);
	left_shoulder_button = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/left_shoulder_button/state", 1);
	right_shoulder_button = n.advertise<baxter_core_msgs::DigitalIOState>
			("/robot/digital_io/right_shoulder_button/state", 1);
    QNode::left_arm_nav.buttons[0]=false;
    QNode::left_arm_nav.buttons[1]=false;
    QNode::left_arm_nav.buttons[2]=false;
    QNode::right_arm_nav.buttons[0]=false;
    QNode::right_arm_nav.buttons[1]=false;
    QNode::right_arm_nav.buttons[2]=false;
    QNode::left_shoulder_nav.buttons[0]=false;
    QNode::left_shoulder_nav.buttons[1]=false;
    QNode::left_shoulder_nav.buttons[2]=false;
    QNode::right_shoulder_nav.buttons[0]=false;
    QNode::right_shoulder_nav.buttons[1]=false;
    QNode::right_shoulder_nav.buttons[2]=false;
    QNode::left_arm_nav.wheel=0;
    QNode::right_arm_nav.wheel=0;
    QNode::left_shoulder_nav.wheel=0;
    QNode::right_shoulder_nav.wheel=0;
    QNode::left_cuff_squeeze.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
    QNode::right_cuff_squeeze.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
    QNode::left_cuff_ok.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
    QNode::right_cuff_ok.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
    QNode::left_cuff_grasp.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
    QNode::right_cuff_grasp.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
    QNode::left_shoulder.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
    QNode::right_shoulder.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
	start();
	return true;
}


void QNode::run() {
	ros::Rate loop_rate(10);
	while ( ros::ok() ) {

    left_itb.publish(QNode::left_arm_nav);
    right_itb.publish(QNode::right_arm_nav);
    torso_left_itb.publish(QNode::left_shoulder_nav);
    torso_right_itb.publish(QNode::right_shoulder_nav);

    left_lower_cuff.publish(QNode::left_cuff_squeeze);
    right_lower_cuff.publish(QNode::right_cuff_squeeze);

    left_lower_button.publish(QNode::left_cuff_ok);
    right_lower_button.publish(QNode::right_cuff_ok);
   
    left_upper_button.publish(QNode::left_cuff_grasp);
    right_upper_button.publish(QNode::right_cuff_grasp); 

    left_shoulder_button.publish(QNode::left_shoulder);
    right_shoulder_button.publish(QNode::right_shoulder); 
                        ros::spinOnce();
                        loop_rate.sleep();
	}
	ROS_INFO("Ros shutdown, proceeding to close the gui.");
}

}  // namespace baxter_sim_io
