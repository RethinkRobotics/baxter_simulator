baxter_simulator
==============

Gazebo simulation with emulated interfaces for the Baxter Research Robot

Code & Tickets
--------------

+-----------------+----------------------------------------------------------------+
| Documentation   | https://github.com/RethinkRobotics/sdk-docs/wiki               |
+-----------------+----------------------------------------------------------------+
| Issues          | https://github.com/RethinkRobotics/baxter_simulator/issues     |
+-----------------+----------------------------------------------------------------+
| Contributions   | https://github.com/RethinkRobotics/sdk-docs/wiki/Contributions |
+-----------------+----------------------------------------------------------------+

baxter_simulator Repository Overview
------------------------------------

::

     .
     |
     +-- baxter_simulator/        baxter_simulator metapackage
     |
     +-- baxter_gazebo/           Gazebo interface for the Baxter that loads the models into simulation
     |   +-- src/
     |   +-- launch/
     |   +-- worlds/
     |
     +-- baxter_sim_controllers/  Controller plugins for Baxter
     |   +-- src/
     |   +-- include/
     |   +-- config/
     |
     +-- baxter_sim_hardware/     This emulates the hardware interfaces of Baxter 
     |   +-- src/
     |   +-- include/
     |   +-- config/
     |   +-- launch/
     |
     +-- baxter_sim_io/           QT based navigator plugins for baxter
     |   +-- src/
     |   +-- include/
     |   +-- ui/
     |
     +-- baxter_sim_kinematics/     Implementation of IK, FK and gravity compensation for baxter 
     |   +-- src/
     |   +-- include/
     |   +-- launch/



Other Baxter Repositories
-------------------------

+------------------+-----------------------------------------------------+
| baxter           | https://github.com/RethinkRobotics/baxter           |
+------------------+-----------------------------------------------------+
| baxter_interface | https://github.com/RethinkRobotics/baxter_interface |
+------------------+-----------------------------------------------------+
| baxter_tools     | https://github.com/RethinkRobotics/baxter_tools     |
+------------------+-----------------------------------------------------+
| baxter_examples  | https://github.com/RethinkRobotics/baxter_examples  |
+------------------+-----------------------------------------------------+
| baxter_common    | https://github.com/RethinkRobotics/baxter_common    |
+------------------+-----------------------------------------------------+

Latest Release Information
--------------------------

https://github.com/RethinkRobotics/sdk-docs/wiki/Release-Changes
