Software in the Loop Joystick Flight
=====

This tutorial explains the steps required to fly a simulated quadcopter in the Gazebo simulator using a real joystick. The following diagram shows how the system components work together.

.. image:: ../_static/sitl_diagram.png
   :scale: 50 %
   :align: center


Hardware Requirements
-----

* Desktop Linux Machine with minimum of 8GB RAM, 16GB recommended, Ubuntu 16.04 installed
* Joystick

.. image:: ../_static/joystick.png
   :scale: 50 %
   :align: center

Software Requirements
-----

* **Ubuntu 16.04**
* **ROS Kinetic** \(full desktop installation\)
* **Gazebo 7**: will be automatically installed with ROS

* **PX4 firmware** installation on Linux: Autopilot software which includes the software-in-the-loop firmware

* **MAVROS** package: Autopilot ROS interface

* **Joy** package: Joystick ROS interface

.. note::
  
  In this tutorial, it is assumed that the reader is familiar with basic Linux commands, ROS Basics.

Setup Steps
-----

* Download this `ZIP file <https://github.com/luym11/initial_settings/archive/master.zip>`_, and extract ``.sh`` files to your home folder and run by command.

.. code-block:: bash
  
  ./ubuntu_install.sh

Then copy commands line by line from ``ws.sh`` and run them one by one in a terminal.

This will setup all permissions and development environment which includes the software-in-the-loop simulation. 

* Install ``QGroundControl`` from `here <https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html#ubuntu-linux>`_. Use the AppImage option.

Testing SITL with Gazebo \(No ROS\)
-----

In this step, we will validate that the PX4 SITL app and Gazebo work as expected. To run the SITL app and Gazebo, execute the following commands in a new terminal

.. code-block:: bash

  cd ~/src/Firmware
  make posix_sitl_default gazebo

After sometime, you should be able to see an Iris model loaded in Gazebo, and the ``pxh>`` command line in the terminal. Just hit ENTER couple of times if you don't see the ``pxh>`` command line, and it should appear.  

To takeoff/land the quadcopter, execute the following commands in the terminal

.. code-block:: bash

  pxh> commander takeoff
  pxh> commander land


If the previous actions succeed the the installation is OK. Next, we will run ROS and a MAVROS node which will allow us to interface the autopilot with ROS.

Interfacing with ROS
-----

Now, you are ready to launch Gazebo+PX4 SITL app+ROS+MAVROS. To do that, execute the following command.

.. code-block:: bash
  
  roslaunch px4 mavros_posix_sitl.launch fcu_url:="udp://:14540@127.0.0.1:14557"

You should be able to see ``/mavros`` topics using ``rostopic list`` in a new terminal. Also if you execute ``rosnode list`` in a new terminal, you should see

.. code-block:: bash

  $ rosnode list
  /gazebo
  /mavros
  /rosout


To double check that MAVROS node is connected properly to the PX4 SITL app, try to ``echo`` some topics _e.g._

.. code-block:: bash

  rostopic echo /mavros/state

Which will show if the mavros node is connected to the PX4 SITL app or not.

Now, you can monitor the drone's states and control it via a MAVROS node.

* As mentioned, in this tutorial, we are going to learn one basic way of controlling the quadcopter's position via a joystick.

* There is a flight mode in PX4 autopilot which is called **OFFBOARD** mode. This mode allows the autopilot to accept specific external commands such as position, velocity, and attitude setpoints. You cannot mix between different setpoints _e.g._ velocity setpoints in x/y and position in z.

* A MAVROS node provides setpoint plugins which will listen to a user input on specific setpoint topics. Once the user publishes to those specific setpoint topics, the mavros node will transfer those setpoints to the autopilot to execute.

* If the autopilot's flight mode is **OFFBOARD**, the autopilot will accept the received setpoints and execute them.

* We will send position setpoints to the autopilot via a setpoint topic that is available in MAVROS. Once set points are received in that topic, the mavros node will send it to the autopilot.

* The setpoint topic that we will use in this tutorial is ``/mavros/setpoint_raw/local``. This topic accepts both position and velocity setpoints according to a specific flag. Next, we will create our custom simple ROS package in which we create a simple ROS node that listens to joystic commands from a ROS topic. Then, it will convert joystic commands to position setpoints which will be published to the ``/mavros/setpoint_raw/local`` topic. Finally, MAVROS will take the position set points and send them to the autopilot.

You might be asking, how are we going to get the joystick commands? The next section explains that.

Joystick Package Installation & Usage
-----

A package named ``joy`` is going to be used to interface a joystick to ROS. To install that package, simply execute the following command in the terminal.

.. code-block:: bash
  
  sudo apt-get install ros-kinetic-joy


You will need to setup permissions before you can use your joystick.

* Plug a joystick

* Check if Linux recognizes your joystick

.. code-block:: bash
  
  ls /dev/input/
  

You will get an output similar to the following.

.. code-block:: bash

  by-id    event0  event2  event4  event6  event8  mouse0  mouse2  uinput
  by-path  event1  event3  event5  event7  js0     mice    mouse1


As you can see, the joystick device is referred to as ``jsX`` where ``X`` is the number of the joystick device.

Let's make the joystick accessible to the joy ROS node.

.. code-block:: bash

  ls -l /dev/input/jsX


You will see something similar to:

.. code-block:: bash

  crw-rw-XX- 1 root dialout 188, 0 2009-08-14 12:04 /dev/input/jsX


If XX is ``rw``: the js device is configured properly. If XX is ``--``: the js device is not configured properly and you need to:

.. code-block:: bash

  sudo chmod a+rw /dev/input/jsX


Test the ``joy`` node. First, start ``roscore`` in a terminal. In another terminal,

.. code-block:: bash

  # set the joystick device address
  rosparam set joy_node/dev "/dev/input/js0"
  # run the joy node
  rosrun joy joy_node


In another terminal, echo the ``joy`` topic and move the joystick to see the topic changes

.. code-block:: bash

  rostopic echo /joy


You should see an output similar to the following.

.. code-block:: bash

  header: 
  seq: 699
  stamp: 
    secs: 1505985329
    nsecs: 399636113
  frame_id: ''
  axes: [-0.0, -0.0, -0.8263657689094543]
  buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


Now, let's write a custom node that reads joystick's commands and convert them to position setpoints to control the quadcopter's poisiton in Gazebo.

Custom Setpoint Node
-----

**Now, it's time for some coding!** You will write a ROS node in Python that listens to the ``/joy`` topic that is published by the ``joy`` node, and converts the joystick commands to xyz position setpoints. Then, it will publish the calculated position setpoints into ``/mavros/setpoint_raw/local``

Publishing to ``/mavros/setpoint_raw/local`` topic is not enough to get the autopilot to track the setpoints. It has to be in **OFFBOARD** mode. So, in your custom node, you will have to send a signal to activate this mode, only once. You need to **remember** that for this mode to work, you will need to be publishing setpoints beforehand, then, activate it, and continue publishing setpoints. **If you don't publish setpoints at more than 2Hz, it will go into a failsafe mode**.

First, create your custom ROS package. The code is commented so you can get an idea of what each part does.


.. code-block:: bash

  cd ~/catkin_ws/src
  catkin_create_pkg mypackage std_msgs mavros_msgs roscpp rospy
  cd mypackage
  # usually python scripts (nodes) are placed in a folder called scripts
  mkdir scripts
  cd scripts
  wget

Make the python file an executable,

.. code-block:: bash

  chmod +x setpoints_node.py


Make a **launch** folder. We will create a ROS launch file to run everything at once.

.. code-block:: bash

  cd ~/catkin_ws/src/mypackage
  mkdir launch
  cd launch
  wget 

In a fresh terminal, you can run the whole system by executing

.. code-block:: bash

  roslaunch mypackage joystick_flight.launch


Now, you should see a quadcopter in Gazebo flying at a fixed height and responding to your joystick commands.

.. warning:: 

  Always make sure that you have joystick permissions configured properly.



