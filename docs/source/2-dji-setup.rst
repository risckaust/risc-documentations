DJI M100 setup
=====

Setting up the DJI M100 with on-board computer(Before you do these steps you must ensure that your drone is binded with your transmitter, your drone is activated and your on-board computer is connected to the drone through the UART port https://developer.dji.com/onboard-sdk/documentation/development-workflow/hardware-setup.html


* Use this link to download and install the DJI SDK to your on board computer (use instructions for ROS): 
https://developer.dji.com/onboard-sdk/documentation/sample-doc/sample-setup.html

* You will have a main ``dji_sdk.launch`` file that runs all sdk features, you need to edit this launch file with the APP ID and APP key and the appropriate Baud rate. You might want to download the DJI Assistant app on a Windows to set some parameters like the baud rate (that needs to match the launch file) or to run simulation. To generate an APP ID and key you will need to register on the DJI website as a developer and activate your account through email. The registration process is a two steps process where in the second step you will need to provide a phone number or credit card info. The second step might require you to press on resend the activation email many times and go again in your email and activate. After you are done with the second step of registration you can now create an APP to generate an IPP ID and key.

* Your drone needs to be in “function mode” (mode is changed from transmitter) and you need to launch the main sdk launch file to start using DJI topics and services. The launch file won't launch without an appropriate APP ID and key. Also you might have an error with the drone activation so you might need to connect your transmitter to a phone that has the DJI account that you activated your drone with before running the launch file. The phone must have the DJI go App. The APP helps in calibrating sensors and receiving camera feedback ( there must be a way to receive camera live streaming through ROS also)

* If you want to have permission to publish to control the DJI drone with SDK you will need to call a ros service ``/dji_sdk/sdk_control_authority`` with boolean arguments (1 for authority).

* If you want to publish local position information to the ``/dji_sdk/local_position`` topic you will need to call the ``/dji_sdk/set_local_pos_ref`` service

* If you want to take-off or land you can call the ``/dji_sdk/drone_task_control`` with argument **4** for take-off and **6** for landing

* You can publish GPS setpoints to the ``/dji_sdk/flight_control_setpoint_ENUposition_yaw`` topic after converting them to **ENU**. Install ``pymap3d`` package by typing 

.. code-block:: bash

    sudo pip install pymap3d==1.6.3

You can import a function from there called ``geodetic2enu``.

* More info about topics and services here: http://wiki.ros.org/dji_sdk  

* A sample code for gps navigation along with a launch file that automatically runs the DJI main node and the required services is available on the RISC Github page.