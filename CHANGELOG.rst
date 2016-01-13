1.2.12 (2016-1-11)
---------------------------------
- This update fixes the conditional yaml-cpp preprocessor define
- Fixed deprecation warning in baxter_sim_kinematics Transforms functions

1.2.11 (2016-1-10)
---------------------------------
- This update is to only compile yaml-cpp functions with preprocessor define
  if the detected system version is yaml-cpp 0.5 or greater. This prevents grippers
  from working in EOL Ubuntu Saucy, but allows the ros_buildfarm to compile.

1.2.1 (2016-1-6)
---------------------------------
- This update is to fix the Isolated Install builds required by the ROS Buildfarm
- Added yaml-cpp as a dependency for baxter_sim_controllers
- Properly exported the include directory of baxter_sim_kinematics
  so that baxter_sim_hardware could utilize its header files
- Properly added install targets for executables in baxter_sim_kinematics
  and baxter_sim_hardware
- Updated headers to properly use / include OpenCV2 dependencies

1.2.0 (2015-12-21)
---------------------------------
- Added a timestamp to the endpoint state topic (thanks @jarvisshultz for contributing)
- Added Signal Handlers to catch Ctrl+C in baxter_sim_io nodes
- General baxter_sim_io code cleanup
- Added run depends for baxter_simulator meta package for all packages in this repo
- Fixed invalid world -> base transform quaternion (thanks @JohnPeterson for contributing)
- Changed Navigator topics to be in sync with baxter_core_msgs
- Added Electric Gripper controllers and emulation (thanks @k-okada for contributing the first pass)
- Improved the fidelity and accuracy of movement by re-tuning the PID loops for the joints (thanks @davetcoleman for the hints)
- Electric Grippers are optional, and can be enabled by setting the top level <side>_electric_gripper for baxter_world.launch
- Added the Simulated IK Pick and Place demo to demonstrate grippers and spawning Gazebo Objects
- Made robot_description an optional arg for baxter_world, in case you have already spawned a custom robot_description
- Fixed an issue in the Position Kinematics node that would cause it to fail if a joint with <left/right> in its name was added to /robot/joint_states

0.9.0 (2015-1-2)
---------------------------------
- Updates baxter_simulator to ROS Indigo (thanks @rethink-hmalaichamee)
- Moves some include files to better support catkin_tools builds (thanks @davetcoleman for contributing)
- Changes rosinstall file to use HTTPS rather than SSH for wstool source management for baxter_simulator

0.8.1 (2014-12-4)
---------------------------------
- Fixes simulated inverse kinematics service to compute IK from <side>_gripper rather than <side>_wrist
- Simplifies forward kinematics function to compute FK for just the endpoint rather than every joint
