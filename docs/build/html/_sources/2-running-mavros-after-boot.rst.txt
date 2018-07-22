Running MAVROS after system boot
=====


Sometimes, It is more convenient to run ROS launch files automatically after robot's computer boots up. For example, if you are working with multiple drones in a swarm, it is painful to log into each drone and run mavros manually. So, to solve this issue, we can run mavros automatically after system boots.  This tutorial shows you one way on how to run ROS launch file after system starts.



Create a simple systemd service
------

Create ``mavros.service`` file in ``/lib/systemd/system``

.. code-block:: bash

	cd /lib/systemd/system
	sudo nano mavros.service

Add the following contents:

.. code-block:: bash

	[Unit]
	Description=mavros 

	[Service]
	Type=forking
	ExecStart=/bin/bash -c "source /opt/ros/kinetic/setup.bash; /usr/bin/python /opt/ros/kinetic/bin/roslaunch mavros px4.launch"
	Restart=on-failure

	[Install]
	WantedBy=multi-user.target

Save and exit by hitting ``CTRL+X``, then ``Y``, then ``[ENTER]``

Then run:

.. code-block:: bash

	sudo systemctl daemon-reload


And enable it on boot:

.. code-block:: bash

	sudo systemctl enable mavros.service


Then, reboot and ``px4.launch`` should be executed after boot.


