baxter_simulator
=============
RSDK Wiki: http://github.com/RethinkRobotics/sdk-docs/wiki

## Repository Description
Necessary files for the Gazebo™ simulation of the Baxter Research Robot from Rethink Robotics Inc.


## Prequisites

 * Setup Github - the git@github.com urls, below, only work if you 
   have [Setup Github](https://help.github.com/articles/set-up-git) and 
   generated [SSH Keys for Github](https://help.github.com/articles/generating-ssh-keys).

 * Ensure the following software packages are installed:

```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python-wstool python-rosdep ros-groovy-pcl-conversions ros-groovy-control-msgs ros-groovy-cmake-modules ros-groovy-moveit-full ros-groovy-driver-common ros-groovy-image-common ros-groovy-rostest gazebo

You have already downloaded and installed the Rethink Robotics SDK into a catkin workspace
```

## Baxter Installation

* From your catkin workspace where the SDK resides, use wstool to install and update:

```
    $ cd ~/ros_ws/src
    $ git clone git@github.com:RethinkRobotics/baxter_simulator.git
    $ wstool merge baxter_simulator/baxter_simulator.rosinstall
    $ wstool update
```


* Known Issues

If you get the following error when doing the catkin_make step below:
```
   CMake Error at /opt/ros/groovy/share/catkin/cmake/catkinConfig.cmake:72 (find_package):
   Could not find a configuration file for package xacro.

   Set xacro_DIR to the directory containing a CMake configuration file for
   xacro.  The file will have one of the following names:

    xacroConfig.cmake
    xacro-config.cmake
```

You should build xacro from source using the hydro branch:

```
    $ cd ~/ros_ws/src
    $ git clone https://github.com/ros/xacro.git -b hydro-devel
```

* Build:

```
    $ source /opt/ros/groovy/setup.bash
    $ cd ~/ros_ws
    $ catkin_make
    $ catkin_make install 
```

* Use baxter.sh - it has a special hook for sim:


```
    $ cp src/baxter/baxter.sh .

```

###Edit the your_ip value in baxter.sh

* Run the script with sim specified:

```
    $ ./baxter.sh sim

```



### Simulation 

 * Start simulation with controllers:
   ```
   $ roslaunch baxter_gazebo baxter_world.launch
   ```
   By default the position controllers are started. To switch, use the JointCommand topic 
   as documented in the Baxter SDK.

 * Optional: Test/tune the velocity controllers or position controllers using a RQT dashboard GUI. 
   Make sure you are in the right joint command mode when using these:

   ```
   $ roslaunch baxter_control baxter_sdk_position_rqt.launch
   ```
   or
   ```
   $ roslaunch baxter_control baxter_sdk_velocity_rqt.launch 
   ```

## Run SDK Examples

NOTE: Before running any examples you need to change the JOINT_ANGLE_TOLERANCE setting in the ‘ros_ws/src/baxter_interface/src/baxter_interface/settings.py’ file:

change ‘JOINT_ANGLE_TOLERANCE = 0.008726646’ to ‘JOINT_ANGLE_TOLERANCE = 0.08726646’

Which changes the value from 0.5 degrees to 5 degrees across each joint.

 * Start Wobbler example:

   ```
   $ ./baxter.sh sim
   $ ~/ros_ws/src/baxter_simulator/baxter_spoof.sh
   $ rosrun baxter_examples joint_velocity_wobbler.py
   ```

 * Start keyboard joint position example:

   ```
   $ ./baxter.sh sim
   $ ~/ros_ws/src/baxter_simulator/baxter_spoof.sh
   $ rosrun baxter_examples joint_position_keyboard.py

   ```


## Start MoveIt

Works with simulation or hardware:

 * Check out the Baxter MoveIt configuration package into your ROS workspace and rebuild:

   ```
   $ cd ~/ros_ws/src
   $ git clone https://github.com/ros-planning/moveit_robots.git
   $ source /opt/ros/groovy/setup.bash
   $ cd ..
   $ catkin_make
   $ catkin_make install 
   ```

 * Start Trajectory Controller:

   ```
   $ rosrun baxter_interface joint_trajectory_action_server.py
   ```

 * Start MoveIt:

   ```
   $ roslaunch baxter_moveit_config demo_baxter.launch
   ```



