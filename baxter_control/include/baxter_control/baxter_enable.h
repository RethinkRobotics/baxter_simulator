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
#include "baxter_core_msgs/AssemblyState.h"
#include "baxter_core_msgs/EndEffectorState.h"
#include "baxter_core_msgs/EndEffectorProperties.h"
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace baxter_en {

class baxter_enable {

  public:
	/**
 	* Method to initialize the default values for all the variables, instantiate the publishers and 	* subscribers
 	* @param img_path that refers the path of the image that loads on start up
 	*/
	bool init(const std::string &img_path);

  private:
	ros::Subscriber enable_sub_, stop_sub_,reset_sub_;
	ros::Publisher assembly_state_pub_, left_grip_st_pub_, right_grip_st_pub_, 
			left_grip_prop_pub_, right_grip_prop_pub_;

	baxter_core_msgs::AssemblyState assembly_state_;
	baxter_core_msgs::EndEffectorState left_grip_st, right_grip_st;
	baxter_core_msgs::EndEffectorProperties left_grip_prop, right_grip_prop;

	ros::Timer timer_;

	/**
 	* Method to publish the loading image on baxter's screen and other publishers that were instantiated
 	* @param Nodehandle to initialize the image transport
 	* @param img_path that refers the path of the image that loads on start up
 	*/
	void publish(ros::NodeHandle &n,const std::string &img_path);
	
	/**
 	* Callback function to enable the robot
 	*/
	void enable_cb(const std_msgs::Bool &msg);

	/**
 	* Callback function to stop the robot and capture the source of the stop
 	*/
	void stop_cb(const std_msgs::Empty &msg);

	/**
	* Callback function all the values to False and 0s
 	*/
	void reset_cb(const std_msgs::Empty &msg);

};

} // namespace

#endif /* BAXTER_ENABLE_H_ */
