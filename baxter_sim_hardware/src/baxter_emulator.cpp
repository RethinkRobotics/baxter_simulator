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
 *  \desc   Node emulating the Baxter hardware interfaces for simulation
 *		commands
 */

#include <baxter_sim_hardware/baxter_emulator.h>

namespace baxter_en {

// Topics to subscribe and publish
static const std::string BAXTER_STATE_TOPIC = "robot/state";
static const std::string BAXTER_ENABLE_TOPIC = "robot/set_super_enable";
static const std::string BAXTER_STOP_TOPIC = "robot/set_super_stop";
static const std::string BAXTER_RESET_TOPIC = "robot/set_super_reset";
static const std::string BAXTER_DISPLAY_TOPIC = "robot/xdisplay";

static const std::string BAXTER_LEFT_GRIPPER_ST =
    "robot/end_effector/left_gripper/state";
static const std::string BAXTER_RIGHT_GRIPPER_ST =
    "robot/end_effector/right_gripper/state";
static const std::string BAXTER_LEFT_GRIPPER_PROP =
    "robot/end_effector/left_gripper/properties";
static const std::string BAXTER_RIGHT_GRIPPER_PROP =
    "robot/end_effector/right_gripper/properties";
static const std::string BAXTER_JOINT_TOPIC = "robot/joint_states";
static const std::string BAXTER_LEFT_LASER_TOPIC =
    "robot/laserscan/left_hand_range/state";
static const std::string BAXTER_RIGHT_LASER_TOPIC =
    "robot/laserscan/right_hand_range/state";
static const std::string BAXTER_LEFT_IR_TOPIC = "robot/range/left_hand_range";
static const std::string BAXTER_RIGHT_IR_TOPIC = "robot/range/right_hand_range";
static const std::string BAXTER_LEFT_IR_STATE_TOPIC =
    "robot/analog_io/left_hand_range/state";
static const std::string BAXTER_RIGHT_IR_STATE_TOPIC =
    "robot/analog_io/right_hand_range/state";
static const std::string BAXTER_LEFT_IR_INT_TOPIC =
    "robot/analog_io/left_hand_range/value_uint32";
static const std::string BAXTER_RIGHT_IR_INT_TOPIC =
    "robot/analog_io/right_hand_range/value_uint32";

static const std::string BAXTER_NAV_LIGHT_TOPIC = "robot/digital_io/command";
static const std::string BAXTER_LEFTIL_TOPIC =
    "robot/digital_io/left_itb_light_inner/state";
static const std::string BAXTER_LEFTOL_TOPIC =
    "robot/digital_io/left_itb_light_outer/state";
static const std::string BAXTER_TORSO_LEFTIL_TOPIC =
    "robot/digital_io/torso_left_itb_light_inner/state";
static const std::string BAXTER_TORSO_LEFTOL_TOPIC =
    "robot/digital_io/torso_left_itb_light_outer/state";
static const std::string BAXTER_RIGHTIL_TOPIC =
    "robot/digital_io/right_itb_light_inner/state";
static const std::string BAXTER_RIGHTOL_TOPIC =
    "robot/digital_io/right_itb_light_outer/state";
static const std::string BAXTER_TORSO_RIGHTIL_TOPIC =
    "robot/digital_io/torso_right_itb_light_inner/state";
static const std::string BAXTER_TORSO_RIGHTOL_TOPIC =
    "robot/digital_io/torso_right_itb_light_outer/state";

static const std::string BAXTER_HEAD_STATE_TOPIC = "robot/head/head_state";
static const std::string BAXTER_HEAD_NOD_CMD_TOPIC =
    "robot/head/command_head_nod";

static const int IMG_LOAD_ON_STARTUP_DELAY = 35;  // Timeout for publishing a single RSDK image on start up

enum nav_light_enum {
  left_itb_light_inner,
  right_itb_light_inner,
  torso_left_itb_light_inner,
  torso_right_itb_light_inner,
  left_itb_light_outer,
  right_itb_light_outer,
  torso_left_itb_light_outer,
  torso_right_itb_light_outer
};

std::map<std::string, nav_light_enum> nav_light;
/**
 * Method to initialize the default values for all the variables, instantiate the publishers and subscribers
 */
bool baxter_emulator::init() {

//Default values for the assembly state
  assembly_state.enabled = false;             // true if enabled
  assembly_state.stopped = false;          // true if stopped -- e-stop asserted
  assembly_state.error = false;  // true if a component of the assembly has an error
  assembly_state.estop_button =
      baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;  // button status
  assembly_state.estop_source =
      baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;  // If stopped is 										true, the source of the e-stop.

  //Default values for the left and right gripper end effector states
  left_grip_st.timestamp.sec = 0;
  left_grip_st.timestamp.nsec = 0;
  left_grip_st.id = 1;
  left_grip_st.enabled = 1;
  left_grip_st.calibrated = 1;
  left_grip_st.ready = 1;
  left_grip_st.moving = 0;
  left_grip_st.gripping = 0;
  left_grip_st.missed = 0;
  left_grip_st.error = 0;
  left_grip_st.position = 0.0;
  left_grip_st.force = 0.0;
  left_grip_st.state = "sample";
  left_grip_st.command = "no_op";
  left_grip_st.command_sender = "";
  left_grip_st.command_sequence = 0;

  right_grip_st = left_grip_st;  // Sample values recorded on both the grippers to do the spoof

  //Default values for the left and the right gripper properties
  left_grip_prop.id = 65664;
  left_grip_prop.ui_type = 2;
  left_grip_prop.manufacturer = "test";
  left_grip_prop.product = "test";
  left_grip_prop.product = "test";
  left_grip_prop.hardware_rev = "test";
  left_grip_prop.firmware_rev = "test";
  left_grip_prop.firmware_date = "test";
  left_grip_prop.controls_grip = true;
  left_grip_prop.senses_grip = true;
  left_grip_prop.reverses_grip = true;
  left_grip_prop.controls_force = true;
  left_grip_prop.senses_force = true;
  left_grip_prop.controls_position = true;
  left_grip_prop.senses_position = true;
  left_grip_prop.properties = "";

  right_grip_prop = left_grip_prop;  // Sample values recorded on both the grippers to do the spoof

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
  head_msg.isPanning = false;
  head_msg.isNodding = false;

  isStopped = false;

// Initialize the map that would be used in the nav_light_cb
  nav_light["left_itb_light_inner"] =left_itb_light_inner;
  nav_light["right_itb_light_inner"] =right_itb_light_inner;
  nav_light["torso_left_itb_light_inner"] =torso_left_itb_light_inner;
  nav_light["torso_right_itb_light_inner"] =torso_right_itb_light_inner;
  nav_light["left_itb_light_outer"] =left_itb_light_outer;
  nav_light["right_itb_light_outer"] =right_itb_light_outer;
  nav_light["torso_left_itb_light_outer"] =torso_left_itb_light_outer;
  nav_light["torso_right_itb_light_outer"] =torso_right_itb_light_outer;

  // Initialize the publishers
  assembly_state_pub = n.advertise<baxter_core_msgs::AssemblyState>(
      BAXTER_STATE_TOPIC, 1);
  left_grip_st_pub = n.advertise<baxter_core_msgs::EndEffectorState>(
      BAXTER_LEFT_GRIPPER_ST, 1);
  right_grip_st_pub = n.advertise<baxter_core_msgs::EndEffectorState>(
      BAXTER_RIGHT_GRIPPER_ST, 1);
  left_grip_prop_pub = n.advertise<baxter_core_msgs::EndEffectorProperties>(
      BAXTER_LEFT_GRIPPER_PROP, 1);
  right_grip_prop_pub = n.advertise<baxter_core_msgs::EndEffectorProperties>(
      BAXTER_RIGHT_GRIPPER_PROP, 1);
  left_ir_pub = n.advertise<sensor_msgs::Range>(BAXTER_LEFT_IR_TOPIC, 1);
  right_ir_pub = n.advertise<sensor_msgs::Range>(BAXTER_RIGHT_IR_TOPIC, 1);
  left_ir_state_pub = n.advertise<baxter_core_msgs::AnalogIOState>(
      BAXTER_LEFT_IR_STATE_TOPIC, 1);
  right_ir_state_pub = n.advertise<baxter_core_msgs::AnalogIOState>(
      BAXTER_RIGHT_IR_STATE_TOPIC, 1);
  left_ir_int_pub = n.advertise<std_msgs::UInt32>(BAXTER_LEFT_IR_INT_TOPIC, 1);
  right_ir_int_pub = n.advertise<std_msgs::UInt32>(BAXTER_RIGHT_IR_INT_TOPIC,
                                                   1);

  left_itb_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_LEFTIL_TOPIC, 1);
  left_itb_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_LEFTOL_TOPIC, 1);
  torso_left_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_TORSO_LEFTIL_TOPIC, 1);
  torso_left_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_TORSO_LEFTOL_TOPIC, 1);

  right_itb_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_RIGHTIL_TOPIC, 1);
  right_itb_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_RIGHTOL_TOPIC, 1);
  torso_right_innerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_TORSO_RIGHTIL_TOPIC, 1);
  torso_right_outerL_pub = n.advertise<baxter_core_msgs::DigitalIOState>(
      BAXTER_TORSO_RIGHTOL_TOPIC, 1);

  head_pub = n.advertise<baxter_core_msgs::HeadState>(BAXTER_HEAD_STATE_TOPIC,
                                                      1);

  // Initialize the subscribers
  enable_sub = n.subscribe(BAXTER_ENABLE_TOPIC, 100,
                           &baxter_emulator::enable_cb, this);
  stop_sub = n.subscribe(BAXTER_STOP_TOPIC, 100, &baxter_emulator::stop_cb,
                         this);
  reset_sub = n.subscribe(BAXTER_RESET_TOPIC, 100, &baxter_emulator::reset_cb,
                          this);
  jnt_st = n.subscribe(BAXTER_JOINT_TOPIC, 100, &baxter_emulator::update_jnt_st,
                     this);
  left_laser_sub = n.subscribe(BAXTER_LEFT_LASER_TOPIC, 100,
                               &baxter_emulator::left_laser_cb, this);
  right_laser_sub = n.subscribe(BAXTER_RIGHT_LASER_TOPIC, 100,
                                &baxter_emulator::right_laser_cb, this);
  nav_light_sub = n.subscribe(BAXTER_NAV_LIGHT_TOPIC, 100,
                              &baxter_emulator::nav_light_cb, this);
  head_nod_sub = n.subscribe(BAXTER_HEAD_NOD_CMD_TOPIC, 100,
                             &baxter_emulator::head_nod_cb, this);
  head_nod_timer = n.createTimer(ros::Duration(1),
                                 &baxter_emulator::reset_head_nod, this, true,
                                 false);
}

/**
 * Method that publishes the emulated interfaces' states and data at 100 Hz
 * @param img_path that refers the path of the image that loads on start up
 */
void baxter_emulator::publish(const std::string &img_path) {
  ros::Rate loop_rate(100);

  arm_kinematics::Kinematics kin;
  kin.init_grav();

  image_transport::ImageTransport it(n);
  image_transport::Publisher display_pub = it.advertise(BAXTER_DISPLAY_TOPIC,
                                                        1);

  // Read OpenCV Mat image and convert it to ROS message
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  try {
    cv_ptr->image = cv::imread(img_path, CV_LOAD_IMAGE_UNCHANGED);
    if (cv_ptr->image.data) {
      cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
      sleep(IMG_LOAD_ON_STARTUP_DELAY);  // Wait for the model to load
      display_pub.publish(cv_ptr->toImageMsg());
    }
  } catch (std::exception &e) {
    ROS_WARN("Unable to load the Startup picture on Baxter's display screen ",e.what());
  }
  ROS_INFO("Simulator is loaded and started successfully");
  while (ros::ok()) {

    assembly_state_pub.publish(assembly_state);
    left_grip_st_pub.publish(left_grip_st);
    right_grip_st_pub.publish(right_grip_st);
    left_grip_prop_pub.publish(left_grip_prop);
    right_grip_prop_pub.publish(right_grip_prop);
    left_ir_pub.publish(left_ir);
    left_ir_state_pub.publish(left_ir_state);
    left_ir_int_pub.publish(left_ir_int);
    left_itb_innerL_pub.publish(leftIL_nav_light);
    left_itb_outerL_pub.publish(leftOL_nav_light);
    torso_left_innerL_pub.publish(torso_leftIL_nav_light);
    torso_left_outerL_pub.publish(torso_leftOL_nav_light);
    right_ir_pub.publish(right_ir);
    right_ir_state_pub.publish(right_ir_state);
    right_ir_int_pub.publish(right_ir_int);
    right_itb_innerL_pub.publish(rightIL_nav_light);
    right_itb_outerL_pub.publish(rightOL_nav_light);
    torso_right_innerL_pub.publish(torso_rightIL_nav_light);
    torso_right_outerL_pub.publish(torso_rightOL_nav_light);
    head_pub.publish(head_msg);
    kin.getGravityTorques(jstate_msg, assembly_state.enabled);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
/**
 * Method to enable the robot
 */
void baxter_emulator::enable_cb(const std_msgs::Bool &msg) {

  if (msg.data && !isStopped) {
    assembly_state.enabled = true;
  }

  else {
    assembly_state.enabled = false;
  }
  assembly_state.stopped = false;
  assembly_state.estop_button =
      baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state.estop_source =
      baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  enable = assembly_state.enabled;
}

/**
 * Method to stop the robot and capture the source of the stop
 */
void baxter_emulator::stop_cb(const std_msgs::Empty &msg) {
  assembly_state.enabled = false;
  assembly_state.stopped = true;
  assembly_state.estop_button =
      baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state.estop_source =
      baxter_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN;
  enable = false;
  isStopped = true;
}

/**
 * Method resets all the values to False and 0s
 */
void baxter_emulator::reset_cb(const std_msgs::Empty &msg) {
  assembly_state.enabled = false;
  assembly_state.stopped = false;
  assembly_state.estop_button =
      baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state.estop_source =
      baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  assembly_state.error = false;
  enable = false;
  isStopped = false;
}

/**
 * Method to capture the laser data and pass it as IR data for the left arm
 */
void baxter_emulator::left_laser_cb(const sensor_msgs::LaserScan &msg) {
  left_ir.header = msg.header;
  left_ir.min_range = msg.range_min;
  left_ir.max_range = msg.range_max;
  left_ir.radiation_type = 1;
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
void baxter_emulator::right_laser_cb(const sensor_msgs::LaserScan &msg) {
  right_ir.header = msg.header;
  right_ir.min_range = msg.range_min;
  right_ir.max_range = msg.range_max;
  right_ir.radiation_type = 1;
  if (msg.ranges[0] < msg.range_max && msg.ranges[0] > msg.range_min)
    right_ir.range = msg.ranges[0];
  else
    right_ir.range = 65.5350036621;
  right_ir_state.timestamp = right_ir.header.stamp;
  right_ir_state.value = right_ir.range / 1000;
  right_ir_state.isInputOnly = true;
  right_ir_int.data = right_ir.range;
}

void baxter_emulator::nav_light_cb(
    const baxter_core_msgs::DigitalOutputCommand &msg) {
  int res;
  if (msg.value)
    res = baxter_core_msgs::DigitalIOState::ON;
  else
    res = baxter_core_msgs::DigitalIOState::OFF;
  switch (nav_light.find(msg.name)->second) {
    case left_itb_light_inner:
      leftIL_nav_light.state = res;
      break;
    case right_itb_light_inner:
      rightIL_nav_light.state = res;
      break;
    case torso_left_itb_light_inner:
      torso_leftIL_nav_light.state = res;
      break;
    case torso_right_itb_light_inner:
      torso_rightIL_nav_light.state = res;
      break;
    case left_itb_light_outer:
      leftOL_nav_light.state = res;
      break;
    case right_itb_light_outer:
      rightOL_nav_light.state = res;
      break;
    case torso_left_itb_light_outer:
      torso_leftOL_nav_light.state = res;
      break;
    case torso_right_itb_light_outer:
      torso_rightOL_nav_light.state = res;
      break;
    default:
      ROS_ERROR("Not a valid component id");
      break;
  }
}

void baxter_emulator::head_nod_cb(const std_msgs::Bool &msg) {
  if (msg.data) {
    head_msg.isNodding = true;
    if (!head_nod_timer.hasPending()) {
      head_nod_timer.setPeriod(ros::Duration(1));
      head_nod_timer.start();
    }
  }
}

void baxter_emulator::reset_head_nod(const ros::TimerEvent &t) {
  head_msg.isNodding = false;
}

void baxter_emulator::update_jnt_st(const sensor_msgs::JointState &msg) {
  jstate_msg = msg;
  float threshold = 0.0009;
  for (int i = 0; i < msg.name.size(); i++) {
    if (msg.name[i] == "head_pan") {
      if (fabs(float(head_msg.pan) - float(msg.position[i])) > threshold)
        head_msg.isPanning = true;
      else
        head_msg.isPanning = false;
      head_msg.pan = msg.position[i];
    }
  }
}

}  //namespace

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "baxter_emulator");

  std::string img_path = argc > 1 ? argv[1] : "";
  baxter_en::baxter_emulator emulate;
  bool result = emulate.init();
  emulate.publish(img_path);

  return 0;
}
