0.9.0 (2015-1-2)
---------------------------------
- Updates baxter_simulator to ROS Indigo (thanks @rethink-hmalaichamee)
- Moves some include files to better support catkin_tools builds (thanks @davetcoleman for contributing)

0.8.1 (2014-12-4)
---------------------------------
- Fixes simulated inverse kinematics service to compute IK from <side>_gripper rather than <side>_wrist
- Simplifies forward kinematics function to compute FK for just the endpoint rather than every joint
