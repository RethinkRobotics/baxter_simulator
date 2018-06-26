/*********************************************************************
 # Copyright (c) 2015, Rethink Robotics
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
 *  \desc   Node emulating the Baxter hardware interfaces for simulation
 *		commands
 */

#include <baxter_sim_hardware/baxter_emulator.h>

namespace baxter_en
{
// Topics to subscribe and publish
const std::string BAXTER_STATE_TOPIC = "robot/state";
const std::string BAXTER_ENABLE_TOPIC = "robot/set_super_enable";
const std::string BAXTER_STOP_TOPIC = "robot/set_super_stop";
const std::string BAXTER_RESET_TOPIC = "robot/set_super_reset";
const std::string BAXTER_DISPLAY_TOPIC = "robot/xdisplay";

const std::string BAXTER_LEFT_GRIPPER_ST = "robot/end_effector/left_gripper/state";
const std::string BAXTER_RIGHT_GRIPPER_ST = "robot/end_effector/right_gripper/state";
const std::string BAXTER_LEFT_GRIPPER_PROP = "robot/end_effector/left_gripper/properties";
const std::string BAXTER_RIGHT_GRIPPER_PROP = "robot/end_effector/right_gripper/properties";
const std::string BAXTER_JOINT_TOPIC = "robot/joint_states";
const std::string BAXTER_LEFT_LASER_TOPIC = "sim/laserscan/left_hand_range/state";
const std::string BAXTER_RIGHT_LASER_TOPIC = "sim/laserscan/right_hand_range/state";
const std::string BAXTER_LEFT_IR_TOPIC = "robot/range/left_hand_range/state";
const std::string BAXTER_RIGHT_IR_TOPIC = "robot/range/right_hand_range/state";
const std::string BAXTER_LEFT_IR_STATE_TOPIC = "robot/analog_io/left_hand_range/state";
const std::string BAXTER_RIGHT_IR_STATE_TOPIC = "robot/analog_io/right_hand_range/state";
const std::string BAXTER_LEFT_IR_INT_TOPIC = "robot/analog_io/left_hand_range/value_uint32";
const std::string BAXTER_RIGHT_IR_INT_TOPIC = "robot/analog_io/right_hand_range/value_uint32";

const std::string BAXTER_NAV_LIGHT_TOPIC = "robot/digital_io/command";
const std::string BAXTER_LEFTIL_TOPIC = "robot/digital_io/left_inner_light/state";
const std::string BAXTER_LEFTOL_TOPIC = "robot/digital_io/left_outer_light/state";
const std::string BAXTER_TORSO_LEFTIL_TOPIC = "robot/digital_io/torso_left_inner_light/state";
const std::string BAXTER_TORSO_LEFTOL_TOPIC = "robot/digital_io/torso_left_outer_light/state";
const std::string BAXTER_RIGHTIL_TOPIC = "robot/digital_io/right_inner_light/state";
const std::string BAXTER_RIGHTOL_TOPIC = "robot/digital_io/right_outer_light/state";
const std::string BAXTER_TORSO_RIGHTIL_TOPIC = "robot/digital_io/torso_right_inner_light/state";
const std::string BAXTER_TORSO_RIGHTOL_TOPIC = "robot/digital_io/torso_right_outer_light/state";

const std::string BAXTER_HEAD_STATE_TOPIC = "robot/head/head_state";
const std::string BAXTER_HEAD_NOD_CMD_TOPIC = "robot/head/command_head_nod";

const std::string BAXTER_LEFT_GRAVITY_TOPIC = "robot/limb/left/gravity_compensation_torques";
const std::string BAXTER_RIGHT_GRAVITY_TOPIC = "robot/limb/right/gravity_compensation_torques";

const std::string BAXTER_SIM_STARTED = "robot/sim/started";

const int IMG_LOAD_ON_STARTUP_DELAY = 10;  // Timeout for publishing a single RSDK image on start up

enum nav_light_enum
{
  left_inner_light,
  right_inner_light,
  torso_left_inner_light,
  torso_right_inner_light,
  left_outer_light,
  right_outer_light,
  torso_left_outer_light,
  torso_right_outer_light
};

std::map<std::string, nav_light_enum> nav_light;
/**
 * Method to initialize the default values for all the variables, instantiate the publishers and subscribers
 */
bool baxter_emulator::init()
{
  // Default values for the assembly state
  assembly_state.enabled = false;  // true if enabled
  assembly_state.stopped = false;  // true if stopped -- e-stop asserted
  assembly_state.error = false;    // true if a component of the assembly has an error
  assembly_state.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;  // button status
  assembly_state.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;       // If stopped is
  // true, the source of the e-stop.

  // Default values for the left and right gripper end effector states
  left_grip_st.timestamp.sec = 0;
  left_grip_st.timestamp.nsec = 0;
  left_grip_st.id = 131073;
  left_grip_st.enabled = 1;
  left_grip_st.calibrated = 1;
  left_grip_st.ready = 1;
  left_grip_st.moving = 0;
  left_grip_st.gripping = 0;
  left_grip_st.missed = 0;
  left_grip_st.error = 0;
  left_grip_st.position = 0.0;
  left_grip_st.force = 0.0;
  left_grip_st.state = "";
  left_grip_st.command = "";
  left_grip_st.command_sender = "";
  left_grip_st.command_sequence = 0;

  right_grip_st = left_grip_st;  // Sample values recorded on both the grippers to do the spoof

  left_grip_prop.id = 131073;
  left_grip_prop.ui_type = 3;
  left_grip_prop.manufacturer = "Rethink Research Robot";
  left_grip_prop.product = "SDK End Effector";
  left_grip_prop.serial_number = "";
  left_grip_prop.hardware_rev = "";
  left_grip_prop.firmware_rev = "";
  left_grip_prop.firmware_date = "";
  left_grip_prop.controls_grip = true;
  left_grip_prop.senses_grip = true;
  left_grip_prop.reverses_grip = true;
  left_grip_prop.controls_force = true;
  left_grip_prop.senses_force = true;
  left_grip_prop.controls_position = true;
  left_grip_prop.senses_position = true;
  left_grip_prop.properties = "";
  std::string gripper_type;
  ros::param::param<std::string>("~left_gripper_type", gripper_type, "PASSIVE_GRIPPER");
  if (gripper_type == "SUCTION_CUP_GRIPPER")
  {
    left_grip_prop.id = 65537;
    left_grip_prop.ui_type = 1;
  }
  else if (gripper_type == "ELECTRIC_GRIPPER")
  {
    left_grip_prop.id = 65538;
    left_grip_prop.ui_type = 2;
    left_grip_prop.serial_number = "3712199347";
    left_grip_prop.hardware_rev = "2";
    left_grip_prop.firmware_rev = "3.0.0 5.5";
    left_grip_prop.firmware_date = "2014/7/24 18:30:00";
  }

  right_grip_prop = left_grip_prop;  // Sample values recorded on both the grippers to do the spoof
  right_grip_prop.serial_number = "";
  right_grip_prop.hardware_rev = "";
  right_grip_prop.firmware_rev = "";
  right_grip_prop.firmware_date = "";
  ros::param::param<std::string>("~right_gripper_type", gripper_type, "PASSIVE_GRIPPER");
  if (gripper_type == "SUCTION_CUP_GRIPPER")
  {
    right_grip_prop.id = 65537;
    right_grip_prop.ui_type = 1;
  }
  else if (gripper_type == "ELECTRIC_GRIPPER")
  {
    right_grip_prop.id = 65538;
    right_grip_prop.ui_type = 2;
    right_grip_prop.serial_number = "3712199347";
    right_grip_prop.hardware_rev = "2";
    right_grip_prop.firmware_rev = "3.0.0 5.5";
    right_grip_prop.firmware_date = "2014/7/24 18:30:00";
  }
  else
  {
    // Default values for the right gripper properties
    right_grip_prop.id = 131073;
    right_grip_prop.ui_type = 3;
  }

  leftIL_nav_light.isInputOnly = false;
  leftOL_nav_light.isInputOnly = false;
  torso_leftIL_nav_light.isInputOnly = false;
  torso_leftOL_nav_light.isInputOnly = false;
  rightIL_nav_light.isInputOnly = false;
  rightOL_nav_light.isInputOnly = false;
  torso_rightIL_nav_light.isInputOnly = false;
  torso_rightOL_nav_light.isInputOnly = false;

  leftIL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;
  leftOL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;
  torso_leftIL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;
  torso_leftOL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;
  rightIL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;
  rightOL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;
  torso_rightIL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;
  torso_rightOL_nav_light.state = baxter_core_msgs::DigitalIOState::OFF;

  head_msg.pan = 0;
  head_msg.isTurning = false;
  head_msg.isNodding = false;

  isStopped = false;

  left_gravity.header.frame_id = "base";
  right_gravity.header.frame_id = "base";

  // Initialize the map that would be used in the nav_light_cb
  nav_light["left_inner_light"] = left_inner_light;
  nav_light["right_inner_light"] = right_inner_light;
  nav_light["torso_left_inner_light"] = torso_left_inner_light;
  nav_light["torso_right_inner_light"] = torso_right_inner_light;
  nav_light["left_outer_light"] = left_outer_light;
  nav_light["right_outer_light"] = right_outer_light;
  nav_light["torso_left_outer_light"] = torso_left_outer_light;
  nav_light["torso_right_outer_light"] = torso_right_outer_light;

  // Initialize the publishers
  assembly_state_pub = n.advertise<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC, 1);
  left_grip_st_pub = n.advertise<baxter_core_msgs::EndEffectorState>(BAXTER_LEFT_GRIPPER_ST, 1);
  right_grip_st_pub = n.advertise<baxter_core_msgs::EndEffectorState>(BAXTER_RIGHT_GRIPPER_ST, 1);
  left_grip_prop_pub = n.advertise<baxter_core_msgs::EndEffectorProperties>(BAXTER_LEFT_GRIPPER_PROP, 1);
  right_grip_prop_pub = n.advertise<baxter_core_msgs::EndEffectorProperties>(BAXTER_RIGHT_GRIPPER_PROP, 1);
  left_ir_pub = n.advertise<sensor_msgs::Range>(BAXTER_LEFT_IR_TOPIC, 1);
  right_ir_pub = n.advertise<sensor_msgs::Range>(BAXTER_RIGHT_IR_TOPIC, 1);
  left_ir_state_pub = n.advertise<baxter_core_msgs::AnalogIOState>(BAXTER_LEFT_IR_STATE_TOPIC, 1);
  right_ir_state_pub = n.advertise<baxter_core_msgs::AnalogIOState>(BAXTER_RIGHT_IR_STATE_TOPIC, 1);
  left_ir_int_pub = n.advertise<std_msgs::UInt32>(BAXTER_LEFT_IR_INT_TOPIC, 1);
  right_ir_int_pub = n.advertise<std_msgs::UInt32>(BAXTER_RIGHT_IR_INT_TOPIC, 1);

  left_inner_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_LEFTIL_TOPIC, 1);
  left_outer_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_LEFTOL_TOPIC, 1);
  torso_left_inner_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_TORSO_LEFTIL_TOPIC, 1);
  torso_left_outer_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_TORSO_LEFTOL_TOPIC, 1);

  right_inner_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_RIGHTIL_TOPIC, 1);
  right_outer_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_RIGHTOL_TOPIC, 1);
  torso_right_inner_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_TORSO_RIGHTIL_TOPIC, 1);
  torso_right_outer_light_pub = n.advertise<baxter_core_msgs::DigitalIOState>(BAXTER_TORSO_RIGHTOL_TOPIC, 1);

  left_grav_pub = n.advertise<baxter_core_msgs::SEAJointState>(BAXTER_LEFT_GRAVITY_TOPIC, 1);
  right_grav_pub = n.advertise<baxter_core_msgs::SEAJointState>(BAXTER_RIGHT_GRAVITY_TOPIC, 1);

  head_pub = n.advertise<baxter_core_msgs::HeadState>(BAXTER_HEAD_STATE_TOPIC, 1);
  // Latched Simulator Started Publisher
  sim_started_pub = n.advertise<std_msgs::Empty>(BAXTER_SIM_STARTED, 1, true);

  // Initialize the subscribers
  enable_sub = n.subscribe(BAXTER_ENABLE_TOPIC, 100, &baxter_emulator::enable_cb, this);
  stop_sub = n.subscribe(BAXTER_STOP_TOPIC, 100, &baxter_emulator::stop_cb, this);
  reset_sub = n.subscribe(BAXTER_RESET_TOPIC, 100, &baxter_emulator::reset_cb, this);
  jnt_st = n.subscribe(BAXTER_JOINT_TOPIC, 100, &baxter_emulator::update_jnt_st, this);
  left_laser_sub = n.subscribe(BAXTER_LEFT_LASER_TOPIC, 100, &baxter_emulator::left_laser_cb, this);
  right_laser_sub = n.subscribe(BAXTER_RIGHT_LASER_TOPIC, 100, &baxter_emulator::right_laser_cb, this);
  nav_light_sub = n.subscribe(BAXTER_NAV_LIGHT_TOPIC, 100, &baxter_emulator::nav_light_cb, this);
  head_nod_sub = n.subscribe(BAXTER_HEAD_NOD_CMD_TOPIC, 100, &baxter_emulator::head_nod_cb, this);
  head_nod_timer = n.createTimer(ros::Duration(1), &baxter_emulator::reset_head_nod, this, true, false);
}

/**
 * Method that publishes the emulated interfaces' states and data at 100 Hz
 * @param img_path that refers the path of the image that loads on start up
 */
void baxter_emulator::startPublishLoop(const std::string& img_path)
{
  ros::Rate loop_rate(100);

  arm_kinematics::Kinematics kin;
  kin.init_grav(); // Disable gravity on Baxter's arms

  image_transport::ImageTransport it(n);
  image_transport::Publisher display_pub = it.advertise(BAXTER_DISPLAY_TOPIC, 1);
  // Read OpenCV Mat image and convert it to ROS message
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  try
  {
    cv_ptr->image = cv::imread(img_path, CV_LOAD_IMAGE_UNCHANGED);
    if (cv_ptr->image.data)
    {
      cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;

      // Wait for the VideoPlugin screen to load, or timeout after delay
      const std::size_t num_required_subscribers = 2;
      waitForSubscriber(display_pub, IMG_LOAD_ON_STARTUP_DELAY, num_required_subscribers);

      display_pub.publish(cv_ptr->toImageMsg());
    }
  }
  catch (std::exception e)
  {
    ROS_WARN_NAMED("emulator", "Unable to load the Startup picture on Baxter's display screen %s", e.what());
  }
  ROS_INFO_NAMED("emulator", "Simulator is loaded and started successfully");

  std_msgs::Empty started_msg;
  sim_started_pub.publish(started_msg);
  while (ros::ok())
  {
    assembly_state_pub.publish(assembly_state);
    left_grip_st_pub.publish(left_grip_st);
    right_grip_st_pub.publish(right_grip_st);
    left_grip_prop_pub.publish(left_grip_prop);
    right_grip_prop_pub.publish(right_grip_prop);
    left_ir_pub.publish(left_ir);
    left_ir_state_pub.publish(left_ir_state);
    left_ir_int_pub.publish(left_ir_int);
    left_inner_light_pub.publish(leftIL_nav_light);
    left_outer_light_pub.publish(leftOL_nav_light);
    torso_left_inner_light_pub.publish(torso_leftIL_nav_light);
    torso_left_outer_light_pub.publish(torso_leftOL_nav_light);
    right_ir_pub.publish(right_ir);
    right_ir_state_pub.publish(right_ir_state);
    right_ir_int_pub.publish(right_ir_int);
    right_inner_light_pub.publish(rightIL_nav_light);
    right_outer_light_pub.publish(rightOL_nav_light);
    torso_right_inner_light_pub.publish(torso_rightIL_nav_light);
    torso_right_outer_light_pub.publish(torso_rightOL_nav_light);
    head_pub.publish(head_msg);
    kin.getGravityTorques(jstate_msg, left_gravity, right_gravity, assembly_state.enabled);
    left_gravity.header.stamp = ros::Time::now();
    left_grav_pub.publish(left_gravity);
    right_gravity.header.stamp = ros::Time::now();
    right_grav_pub.publish(right_gravity);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
/**
 * Method to enable the robot
 */
void baxter_emulator::enable_cb(const std_msgs::Bool& msg)
{
  if (msg.data && !isStopped)
  {
    assembly_state.enabled = true;
  }

  else
  {
    assembly_state.enabled = false;
  }
  assembly_state.stopped = false;
  assembly_state.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  enable = assembly_state.enabled;
}

/**
 * Method to stop the robot and capture the source of the stop
 */
void baxter_emulator::stop_cb(const std_msgs::Empty& msg)
{
  assembly_state.enabled = false;
  assembly_state.stopped = true;
  assembly_state.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN;
  enable = false;
  isStopped = true;
}

/**
 * Method resets all the values to False and 0s
 */
void baxter_emulator::reset_cb(const std_msgs::Empty& msg)
{
  assembly_state.enabled = false;
  assembly_state.stopped = false;
  assembly_state.estop_button = baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state.estop_source = baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  assembly_state.error = false;
  enable = false;
  isStopped = false;
}

/**
 * Method to capture the laser data and pass it as IR data for the left arm
 */
void baxter_emulator::left_laser_cb(const sensor_msgs::LaserScan& msg)
{
  left_ir.header = msg.header;
  left_ir.min_range = msg.range_min;
  left_ir.max_range = msg.range_max;
  left_ir.radiation_type = 1;
  left_ir.field_of_view = 0.0872664600611;
  if (msg.ranges[0] < msg.range_max && msg.ranges[0] > msg.range_min)
    left_ir.range = msg.ranges[0];
  else
    left_ir.range = 65.5350036621;
  left_ir_state.timestamp = left_ir.header.stamp;
  left_ir_state.value = left_ir.range / 1000;
  left_ir_state.isInputOnly = true;
  left_ir_int.data = left_ir.range;
}

/**
 * Method to capture the laser data and pass it as IR data for the right arm
 */
void baxter_emulator::right_laser_cb(const sensor_msgs::LaserScan& msg)
{
  right_ir.header = msg.header;
  right_ir.min_range = msg.range_min;
  right_ir.max_range = msg.range_max;
  right_ir.radiation_type = 1;
  right_ir.field_of_view = 0.0872664600611;
  if (msg.ranges[0] < msg.range_max && msg.ranges[0] > msg.range_min)
    right_ir.range = msg.ranges[0];
  else
    right_ir.range = 65.5350036621;
  right_ir_state.timestamp = right_ir.header.stamp;
  right_ir_state.value = right_ir.range / 1000;
  right_ir_state.isInputOnly = true;
  right_ir_int.data = right_ir.range;
}

void baxter_emulator::nav_light_cb(const baxter_core_msgs::DigitalOutputCommand& msg)
{
  int res;
  if (msg.value)
    res = baxter_core_msgs::DigitalIOState::ON;
  else
    res = baxter_core_msgs::DigitalIOState::OFF;
  switch (nav_light.find(msg.name)->second)
  {
    case left_inner_light:
      leftIL_nav_light.state = res;
      break;
    case right_inner_light:
      rightIL_nav_light.state = res;
      break;
    case torso_left_inner_light:
      torso_leftIL_nav_light.state = res;
      break;
    case torso_right_inner_light:
      torso_rightIL_nav_light.state = res;
      break;
    case left_outer_light:
      leftOL_nav_light.state = res;
      break;
    case right_outer_light:
      rightOL_nav_light.state = res;
      break;
    case torso_left_outer_light:
      torso_leftOL_nav_light.state = res;
      break;
    case torso_right_outer_light:
      torso_rightOL_nav_light.state = res;
      break;
    default:
      ROS_ERROR_NAMED("emulator", "Not a valid component id");
      break;
  }
}

void baxter_emulator::head_nod_cb(const std_msgs::Bool& msg)
{
  if (msg.data)
  {
    head_msg.isNodding = true;
    if (!head_nod_timer.hasPending())
    {
      head_nod_timer.setPeriod(ros::Duration(1));
      head_nod_timer.start();
    }
  }
}

void baxter_emulator::reset_head_nod(const ros::TimerEvent& t)
{
  head_msg.isNodding = false;
}

void baxter_emulator::update_jnt_st(const sensor_msgs::JointState& msg)
{
  jstate_msg = msg;
  float threshold = 0.0009;
  left_gravity.actual_position.resize(left_gravity.name.size());
  left_gravity.actual_velocity.resize(left_gravity.name.size());
  left_gravity.actual_effort.resize(left_gravity.name.size());
  right_gravity.actual_position.resize(left_gravity.name.size());
  right_gravity.actual_velocity.resize(left_gravity.name.size());
  right_gravity.actual_effort.resize(left_gravity.name.size());
  for (int i = 0; i < msg.name.size(); i++)
  {
    if (msg.name[i] == "head_pan")
    {
      if (fabs(float(head_msg.pan) - float(msg.position[i])) > threshold)
        head_msg.isTurning = true;
      else
        head_msg.isTurning = false;
      head_msg.pan = msg.position[i];
    }
    else if (msg.name[i] == "l_gripper_l_finger_joint")
    {
      left_grip_st.position = (msg.position[i] / 0.020833) * 100;
    }
    else if (msg.name[i] == "r_gripper_l_finger_joint")
    {
      right_grip_st.position = (msg.position[i] / 0.020833) * 100;
    }
    else
    {
      for (int j = 0; j < left_gravity.name.size(); j++)
      {
        if (msg.name[i] == left_gravity.name[j])
        {
          left_gravity.actual_position[j] = msg.position[i];
          left_gravity.actual_velocity[j] = msg.velocity[i];
          left_gravity.actual_effort[j] = msg.effort[i];
          break;
        }
        else if (msg.name[i] == right_gravity.name[j])
        {
          right_gravity.actual_position[j] = msg.position[i];
          right_gravity.actual_velocity[j] = msg.velocity[i];
          right_gravity.actual_effort[j] = msg.effort[i];
          break;
        }
      }
    }
  }
}

bool baxter_emulator::waitForSubscriber(const image_transport::Publisher& pub, const double wait_time,
                                        const std::size_t num_req_sub)
{
  // Will wait at most this amount of time
  ros::Time max_time(ros::Time::now() + ros::Duration(wait_time));

  // This is wrong. It returns only the number of subscribers that have already
  // established their direct connections to this publisher
  int num_existing_subscribers = pub.getNumSubscribers();

  // How often to check for subscribers
  ros::Rate poll_rate(200);

  if (wait_time > std::numeric_limits<double>::epsilon() && num_existing_subscribers == 0)
    ROS_INFO_STREAM_NAMED("emulator", "Topic '" << pub.getTopic() << "' waiting for subscriber...");

  // Wait for subscriber
  while (num_existing_subscribers < num_req_sub)
  {
    ROS_DEBUG_STREAM_THROTTLE_NAMED(2.0, "emulator", "Waiting " << max_time - ros::Time::now()
                                    << " num_existing_sub "<< num_existing_subscribers
                                    << " of required " << num_req_sub);


    if (wait_time < std::numeric_limits<double>::epsilon() || ros::Time::now() > max_time)  // Check if timed out
    {
      ROS_WARN_STREAM_NAMED("emulator", "Topic '" << pub.getTopic() << "' unable to connect to " << num_req_sub << " subscribers within "
                            << wait_time << " sec. It is possible initially published visual messages will be lost.");

      return false;
    }
    ros::spinOnce();

    // Sleep
    poll_rate.sleep();

    // Check again
    num_existing_subscribers = pub.getNumSubscribers();

    if (!ros::ok())
      return false;
  }

  return true;
}

}  // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "baxter_emulator");

  std::string img_path = argc > 1 ? argv[1] : "";

  baxter_en::baxter_emulator emulate;
  bool result = emulate.init();
  emulate.startPublishLoop(img_path);

  return 0;
}
