Networking
==========

Case 1: Communication with Parrot SLAM Dunk
--------------------------------------------

Consider the following setup

.. image:: ../_static/slamdunk_network.png
    :width: 600px
    :align: center
    :height: 400px
    :alt: alternate text

* the SLAM DUNK module is connected to ODROID XU4 using Ethernet over USB cable. The SLAM module has the IP ``192.168.45.1``
* The SLAM module runs ``roscore`` and ``ROS_MASTER_URI=http://192.168.45.1:11311``
* ODROID (Ubuntu 16/ROS Kinetic) detects new interface as ``usb0`` and get an assigned IP from SLAM module.
* ODROID also connects to a WiFi router ``192.168.0.1`` through an interface ``wlan0`` with a static IP e.g. ``192.168.0.118``
* The ``usb0`` and the ``wlan0`` interfaces are independent
* There is a ground station PC that is connected to the WiFi router and has a static IP e.g. ``192.168.0.105``

Summary of network devices setup
---------------------------------

SLAMDUNK
^^^^^^^^

* IP: ``192.168.45.1``
* gateway: ``192.168.45.1``
* netmask: ``255.255.255.0``

ODROID
^^^^^^^
* IP (usb0): ``192.168.45.2``
* IP (wlan0): ``192.168.0.118``
* ``ROS_MASTER_URI=http://192.168.45.1:11311``
* ``ROS_HOSTNAME=192.168.45.2``
* Edit ``/etc/hosts``, and add SLAM DUNK host name (``192.168.45.1 slamdunk-00316.local``)

PC
^^^

* IP: ``192.168.0.105``
* gateway: ``192.168.0.1``, wifi router's IP
* netmask: ``255.255.255.0``
* ``ROS_MASTER_URI=http://192.168.45.1:11311``
* ``ROS_HOSTNAME=192.168.0.105``

IP routing
-----------

We need to route between two networks on the ODROID

* Enable ip forward on ODROID:
* in ``/etc/sysctl.conf``, uncomment (or add) ``net.ipv4.ip_forward=1``
* add static route on sLAM DUNK module

.. code-block:: bash

  sudo route add -net 192.168.0.0 netmask 255.255.255.0 gw 192.168.45.2

* Add static route on PC

.. code-block:: bash

  sudo route add -net 192.168.45.0 netmask 255.255.255.0 gw 192.168.0.118


* (Optional):  On ODROID, you can modify ``iptables`` as follows (they will re-set after reboot)

.. code-block:: bash

  sudo iptables -A FORWARD --in-interface usb0 -j ACCEPT
  sudo iptables --table nat -A POSTROUTING --out-interface wlan0 -j MASQUERADE


Check if you can ping all devices to each other. Also, check if you can ``rostopic list`` and ``rostopic echo`` on all three devices.

To make the routing persistent
------------------------------

1. create a script file in the ``/etc/init.d/ folder``.
2. add your route definitions to this file and change it to an executable file (``chmod +x /path/to/file``).
3. run the ``update-rc.d <filename> defaults`` command to make the script executable at boot time.
4. reboot the system and check whether the system adds the routes at startup(``netstat -rn``).