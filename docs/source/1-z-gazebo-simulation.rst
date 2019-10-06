Software in the Loop Rover Control
=====


This tutorial explains the steps required to drive a simulated rover in the Gazebo simulator. We are going to learn a basic way of controlling the rover by publishing the desired setpoints to a specific topic. There is a mode in PX4 autopilot which is called **OFFBOARD** mode. This mode allows the autopilot to accept specific external commands such as position, velocity, or attitude setpoints.

In general, a MAVROS node provides setpoint plugins which will listen to a user input on specific setpoint topics. Once the user publishes to those specific setpoint topics and if the mode set to **OFFBOARD**, the MAVROS node will transfer those setpoints to the autopilot to execute. 


In this tutorial we will send position setpoints to the autopilot via a setpoint topic that is available in MAVROS. Once set points are received in that topic, the MAVROS node will send it to the autopilot. The setpoint topic that we will use in this tutorial is ``mavros/setpoint_position/local``. Next, we will create our custom simple ROS package in which we create a simple ROS node that will publish setpoints one after one to follow the square. Finally, MAVROS will take the position set points and send them to the autopilot to execute.

.. The following diagram shows how the system components work together.

.. .. image:: ../_static/sitl_diagram.png
..    :scale: 50 %
..    :align: center


Hardware Requirements
-----

* Desktop Linux Machine with minimum of 8GB RAM, 16GB recommended, Ubuntu 16.04 installed

Software Requirements
-----

* **Ubuntu 16.04**
* **ROS Kinetic** \(full desktop installation\)
* **Gazebo 7**: will be automatically installed with ROS
* **PX4 firmware** installation on Linux: Autopilot software which includes the software-in-the-loop firmware
* **MAVROS** package: Autopilot ROS interface

.. note::

  In this tutorial, it is assumed that the reader is familiar with basic Linux commands, ROS Basics.

Setup Steps
-----

* Install ``QGroundControl`` from `here <https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html#ubuntu-linux>`_. Use the AppImage option.

SITL with Gazebo
-----

There are launch files available to run the simulation wrapped in ROS

To run SITL wrapped in ROS with Rover configation, the ROS environment needs to be updated:

.. code-block:: bash

  cd ~/src/Firmware
  DONT_RUN=1 make px4_sitl gazebo_rover
  cd ~/catkin_ws
  catkin build

Now close the terminal.

Launching Gazebo with ROS Wrappers
------

Now, you are ready to launch Gazebo + PX4 SITL app + ROS + MAVROS. To do that, execute the following command.

.. code-block:: bash
  
  roslaunch px4 mavros_posix_sitl.launch


If everyting launched properly you should see drone in the simulated environment. In order to change vehicle to rover, relaunch previous command with the specified argument for vehicle. Since default value is iris model drone.

.. code-block:: bash
  
  roslaunch px4 mavros_posix_sitl.launch vehicle:="rover"

You should be able to see many ``/mavros/...`` topics using ``rostopic list`` in a new terminal. Also if you execute ``rosnode list`` in a new terminal, you should see the following

.. code-block:: bash

  $ rosnode list
  /gazebo
  /gazebo_gui
  /mavros
  /rosout


To double check that MAVROS node is connected properly to the PX4 SITL app, try to ``echo`` some topics _e.g._

.. code-block:: bash

  rostopic echo /mavros/state

Which will show if the mavros node is connected to the PX4 SITL app or not.

Now, you can monitor the rover's states and control it via a MAVROS node.

Custom Setpoint Node
-----

**Now, it's time for some coding!** You will write a ROS node in Python that publishes the desired position setpoints into ``mavros/setpoint_position/local``.

Publishing to ``mavros/setpoint_position/local`` topic is not enough to get the autopilot to track the setpoints. It has to be in **OFFBOARD** mode. So, in your custom node, you will have to send a signal to activate this mode, only once. You need to **remember** that for this mode to work, you will need to be publishing setpoints beforehand, then, activate it, and continue publishing setpoints. **If you don't publish setpoints at more than 2Hz, it will go into a failsafe mode** and **OFFBOARD** mode will be off.

First, create your custom ROS package. The code is commented so you can get an idea of what each part does. Go through code and try to understand it!


.. code-block:: bash

  cd ~/catkin_ws/src
  catkin_create_pkg mypackage std_msgs mavros_msgs roscpp rospy
  cd mypackage
  # usually python scripts (nodes) are placed in a folder called scripts
  mkdir scripts
  cd scripts
  wget https://raw.githubusercontent.com/risckaust/risc-documentations/master/src/gazebo-rover/square.py

Make the python file an executable,

.. code-block:: bash

  chmod +x square.py


Make a **launch** folder. We will create a ROS launch file to run everything at once. Open the launch file and understand what every line executes.

.. code-block:: bash

  cd ~/catkin_ws/src/mypackage
  mkdir launch
  cd launch
  wget https://raw.githubusercontent.com/risckaust/risc-documentations/master/src/gazebo-rover/main.launch


Build and source the catkin workspace. In a fresh terminal, you can run the launch file by executing

.. code-block:: bash

  roslaunch mypackage main.launch

Now, you should see a rover following the square autonomously.

Contributors
-----

