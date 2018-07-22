Autostart service after system boot
===================================

Sometimes, It is more convenient to run ROS launch files automatically after robot's computer boots up. For example, if you are working with multiple drones in a swarm, it is painful to log into each drone and run mavros manually. So, to solve this issue, we can run mavros automatically after system boots. This tutorial shows you one way on how to run ROS launch file after system starts.

Create a simple systemd service
--------------------------------

Use case: auto start MAVROS node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
* Create a shell script and type the commands that you would execute in a normal terminal. Fro example,

.. code-block:: bash

	mkdir ~/scripts
	cd ~/scripts
	touch startup_launch.sh
	chmod +x startup_launch.sh

Type the following in the ``startup_launch.sh`` file (you can use the ``nano startup_launch.sh`` command). It is assumed that the username is ``odroid``

.. code-block:: bash

	#!/bin/bash
	source /opt/ros/kinetic/setup.bash
	source /home/odroid/catkin_ws/devel/setup.bash
	roslaunch mavros px4.launch



* Create ``mavros.service`` file in ``/lib/systemd/system``

.. code-block:: bash

	cd /lib/systemd/system
	sudo nano mavros.service

* Add the following contents:

.. code-block:: bash

	[Unit]
	Description=mavros

	[Service]
	Type=forking
	ExecStart=/home/odroid/scripts/startup_launch.sh
	Restart=on-failure

	[Install]
	WantedBy=multi-user.target

Save and exit by hitting ``CTRL+x``, then ``Y``, then ``[ENTER]``

Then run:

.. code-block:: bash

	sudo systemctl daemon-reload

And enable it on boot:

.. code-block:: bash

	sudo systemctl enable mavros.service

Then, reboot and ``px4.launch`` should be executed after boot.

To disable a service,

.. code-block:: bash

	sudo systemctl disable mavros.service