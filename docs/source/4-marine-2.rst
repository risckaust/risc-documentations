How to start bluerov simulator
================================


To setup the environment for Gazebo simulation and BlueROV2 using SITL and Ardusub follow the `following instructions <https://gist.github.com/monabf/bc04b7ab366f812c645bf0aa6f22c8de>`_.

2- run from ardupilot ./was build

3- install mavproxy and mavlink if required https://gist.github.com/monabf/bc04b7ab366f812c645bf0aa6f22c8de


4- run from ArduSub directory: 

.. code-block:: bash
    
    ../Tools/autotest/sim_vehicle.py -f gazebo-bluerov2 -I 0 -j4 -D -L RATBeach --console

5- run from bluerov_ros_playground: 

.. code-block:: bash

    source gazebo.sh
    gazebo --verbose worlds/underwater.world -u

6- add apm.launch to any package you have in ros:

.. code-block:: bash

    source devel/setup.bash
    export LD_LIBRARY_PATH=/opt/ros/kinetic/lib
    roslaunch rplidar_ros apm.launch

7-run:

.. code-block:: bash

    rosrun mavros checkid
    rosrun mavros mavparam set SYSID_MYGCS 1

8-run:

.. code-block:: bash
    
    rosrun mavros mavsafety arm
    rosrun mavros mavsys mode -c MANUAL

by Sarah Toonsi