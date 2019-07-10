Companion Computers Setup
=========


ODROID XU4 setup
------

.. image:: ../_static/odroidxu4.jpg
   :scale: 50 %
   :align: center

List of components
^^^^^^^

* `ODROID XU4 <http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825>`_.
* `16GB \(or more\) eMMc module <http://www.hardkernel.com/main/products/prdt_info.php?g_code=G145628174287>`_.
* `eMMC reader <http://www.hardkernel.com/main/products/prdt_info.php?g_code=G135415955758>`_.
* `Micro\-SD reader <https://www.amazon.com/Computer-Memory-Card-Readers/b?ie=UTF8&node=516872>`_.

* `WiFi Module 3 \(2\.4Ghz only\) <http://www.hardkernel.com/main/products/prdt_info.php?g_code=G137447734369>`_ good for outdoor use,  or `WiFi Module 5 dual band 2\.4\/5Ghz <http://www.hardkernel.com/main/products/prdt_info.php?g_code=G147513281389>`_ good for high-bandwidth. 

* `DC plug cable \- for onboard\/portable power connection <http://www.hardkernel.com/main/products/prdt_info.php?g_code=G141440511056>`_.

Setup Ubuntu
^^^^^^

ODROID XU 4 supports both Ubuntu and Android, see the details on official odroid webpage.

Here we will discuss how to setup Ubuntu 16.04

Flashing Ubuntu image
"""""""

You can use either an SD card or eMMC. eMMC is recommended as it is much faster than SD card, 16GB or more is recommended.


For our applications we use minimal image (minimal, Bare OS) without GUI. Minimal image will have much smaller size and faster boot and less overhead in general. Extract the downloaded image from official odroid webpage and use ``Etcher`` to flash it to either SD or eMMC card.

User account setup
"""""""""""

After downloading and flashing image to odroid, it is recommended to setup a user account for easier handling in the future. Plug eMMC to the odroid, and connect it to the monitor. Login using the root account (user: ``root``, password: ``odroid``).

.. code-block:: bash

	adduser odroid # create new user with name odroid
	adduser odroid sudo # add odroid user to admin group
	adduser odroid dialout # give odroid user access to serial ports

Network Setup
^^^^^^^^^

It is recommended that you use static IP address if you plan to use ODROID via a WiFi network. This will reduce latency over wifi.

To set a static IP address on odroid, do the following.

Open ``/etc/network/interfaces`` file for editing by running following commmand

.. code-block:: bash
	
	sudo nano /etc/network/interfaces

Add or edit the following lines

.. code-block:: bash

	auto wlan0
	# the following will auto-start connection after boot
	allow-hotplug wlan0
	iface wlan0 inet static
	address 192.168.0.xxx # choose a static IP, usually you change the last number only for different devices
	netmask 255.255.255.0
	broadcast 192.168.0.255
	gateway 192.168.0.1 # your router IP
	dns-nameservers 8.8.8.8
	wpa-ssid "wifi_name"
	wpa-psk "wifi_password"

.. note::

	You will need modify ``wlan0`` to match the wifi card number on your odroid once the wifi device is connected. Is possible that it changes when you change the wifi device.

	To check your wifi card number,

	.. code-block:: bash
		
		ifconfig -a

After odroid is connected to WiFi network and internet run the following commands

.. code-block:: bash

	apt-get update
	apt-get upgrade

Reboot the odroid and now login with newly created user.

Installing packages
^^^^^^^^^^^^^^^

Install ROS
""""""""""""""

To install ROS on ODROID follow official instructions from ROS wiki page. We assume that ROS Kinetic is used.

.. important::
	
	Install the ROS-Base: (Bare Bones) not the full desktop version

After installing ROS, you can install ROS packages that you need individually either by using ``apt-get`` or from source.

Install MAVROS
""""""""""""""

.. code-block:: bash

	sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
	wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
	./install_geographiclib_datasets.sh # might require sudo


Install vrpn
""""""""""""""

.. code-block:: bash

	sudo apt-get install ros-kinetic-vrpn-client-ros

After you installed all the packages and software you might want to create an image of the entire eMMC. Plug it into the another Ubuntu running computer and execute the following comands:

.. code-block:: bash

	lsbkl # Will lists the block devices 
	dd if=/dev/sdc of=/path_to_the_folder/backup.img # Match sdc to the eMMC from previous command
	# It will take time to create an image, and will create a file with full capacity of the eMMC
	# To reduce the size and shrink the unused space run the following
	xz -c backup.img > backup.img.xz 


Intel Up Board
--------------

* Up board is used in the Intel Realsense development kit.
* Follow `this guide <https://01.org/developerjourney/recipe/intel-realsense-robotic-development-kit>`_ to setup the Up board

Using Edimax AC600 Wifi module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to install drivers as follows:

.. code-block:: bash

	sudo apt-get update
	git clone https://github.com/gnab/rtl8812au.git
	cd ~/rtl8812au
	make
	sudo make install
	sudo modprobe 8812au

Then, reboot

.. note::
	
	To be able to use ``ssh`` from a remote computer, you will need,
	``sudo apt-get install openssh-server && openssh-client``

Raspberry Pi Setup
---------

.. note::

	To be done.




Intel NUC setup
-------


.. note::

	To be done.