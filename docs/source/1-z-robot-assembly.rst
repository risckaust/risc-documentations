Rover Assembly
=====

Basic principles
-----

Feel free to place the components anywhere inside the frame but take care of wires. Carefully choose zipties, shrinking tubes, double sided tapes or soldering for different situations.

Preliminaries
------

This tutorial assumes you have the following skills:

* :doc:`1-ros-basics`.

* Soldering, if not, please refer to basic skill `video <https://youtu.be/Qps9woUGkvI>`_.


* Basic knowledge about LiPo batteries. Answer the following questions. You may read `this article <https://rogershobbycenter.com/lipoguide/>`_. 

  - What do 3s, 4s mean?
  - What does 20c mean?
  - What does 1400mAh mean?
  - What are the parameters of your battery?
  - How to charge LiPo battery? How to measure it voltage using battery meter?
  - What’s the minimum voltage to use a LiPo on the robot?

.. danger:: Do not leave your battery plugged in your robot for a long time and never discharge a LiPo battery below 3.4V per cell.

Hardware assembly
-----

Introduction
^^^^^

You will need

* Rover frame with four wheels and DC motors.

.. image:: ../_static/rover.jpg
   :scale: 50 %
   :align: center


* Two **motor controllers** to control DC motors.

.. image:: ../_static/motor-controller.png
   :scale: 25 %
   :align: center
 
* Flight Controller. Use any flight controller available in the lab. Just make sure you have compatible power modules, receivers, GPS, and other additional modules. The documentations for each board are available `here <https://docs.px4.io/en/flight_controller/pixhawk_series.html>`_.

* Remote control system. A remote control (RC) radio system is required if you want to manually control your vehicle. In addition to the transmitter/receiver pairs being compatible, the receiver must also be compatible with PX4 and the flight controller hardware. It's recommended to use **Taranis X9D Plus transmitter with X8R receiver** as shown below

.. image:: ../_static/frsky_taranis.jpg
   :scale: 90 %
   :align: center


.. image:: ../_static/x8r.jpg
   :scale: 30 %
   :align: center

* UBEC (Universal Battery eliminator circuit) to convert voltage to power Odroid. A BEC is basically a step down voltage regulator. It will take your main battery voltage (e.g. 11.1 Volts) and reduce it down to ~5 Volts to safely power your Odroid and other electronics.

.. image:: ../_static/ubec.jpg
   :scale: 40 %
   :align: center

* Power module. It is the best way to provide power for flight controller unit. It has voltage and current sensors that allows autopilot to estimate remaining battery charge precisely. Usually it comes with every autopilot controller as a default kit. Check official documentations to match right power module to a selected flight controller.

.. image:: ../_static/power_module.jpg
   :scale: 60 %
   :align: center

* LiPo battery. 3000/4000 mAh 3S battery is recommended.

Assembly process
^^^^^

* Install DC motors to the frame if necessary. Attach the wheels to the motor shafts with provided screws. 

* 

* Assemble the frame. Attach the power distribution board to it (no need if you use frame with soldered pads).


* Connect ESCs to motors and plug ESCs to power distribution board (or solder them to the frame). As for now, connect motors to ESCs arbitrary, later you will set them properly by switching any two wires.



.. image:: ../_static/motor_scheme.jpg
   :scale: 50 %
   :align: center

* Install power module on the frame. One end should be plugged to power distribution board (or soldered to the frame) and the other end to the battery. DON’T plug it to the battery for now.

* Install flight controller on the frame. Take a look at your flight controller and make sure the arrow is pointing to the front between motor 1 and 3. To mount the controller to the frame, use thick double side tape to damp the vibrations.

* Plug cable from power module to ``POWER`` port of your flight controller.

* Plug buzzer and switch to their corresponding ports on flight controller.

* Connect each of your ESCs servo cables to the corresponding **MAIN OUT** output, eg. motor 1 to **MAIN OUT** port 1.

* Binding process for FrSky X8R

    * Connect the RCIN port from Pixhawk to SBUS port on X8R
    * Turn on the X8R while holding the **F/S** button on the module. Release the button.
    * Press the **Menu** button on your Taranis X9D
    * Go to page 2 by pressing **Page** button.
    * Scroll down with **-** button until you see **Internal RF** line.
    * Select **[Bind]** line, and press **ENT** button. The RED LED on the X8R receiver will flash, indicating the binding process is completed

Calibration process
-----

* Download ``QGroundControl`` on your computer and open it.

* `Install Stable PX4 firmware <https://docs.px4.io/en/config/firmware.html>`_.

* Set the airframe to Axial Racing AX10. Follow steps from this `page <https://docs.px4.io/en/config/airframe.html>`_.

* Calibrate `Compass <https://docs.px4.io/en/config/compass.html>`_, `Accelerometer <https://docs.px4.io/en/config/accelerometer.html>`_, and `Level Horizon <https://docs.px4.io/en/config/level_horizon_calibration.html>`_. 
* Calibrate the `Radio <https://docs.px4.io/en/config/radio.html#performing-the-calibration>`_.

* In ``Flight Modes`` tab under the **Flight Mode Settings** and **Switch settings** sections set:

  - **Mode Channel** to SB (SB switch labeled on your Taranis X9D)
  - **Mode 1: Manual**. 
  - **Mode 4: Altitude**. Climb and drop are controlled to have a maximum rate.
  - **Mode 6: Position**. When sticks are released the vehicle will stop and hold position.
  - **Emergency Kill switch channel** to SF (SF switch labeled on your Taranis X9D). Immediately stops all motor outputs. The vehicle will crash, which may in some circumstances be more desirable than allowing it to continue flying.
  - **Offboard switch channel** to SA (SA switch labeled on your Taranis X9D).

You should have similar as shown in the picture below. Channels for **Flight Mode Settings** and **Switch Settings** might differ.

.. image:: ../_static/qground.png
   :scale: 60 %
   :align: center

.. hint::
  
  If you set everything right, you will see changes in **Flight Mode Settings** section highlighted as yellow. Also, moving sticks, dials and switches will be reported in **Channel Monitor** section.

* In ``Power tab`` write the parameters of your battery (Number of cells), calibrate the battery voltage.

  * Press **Calculate** on the **Voltage divider** line
  * Measure the voltage with Digital Battery Capacity Checker by connecting it to the battery
  * Enter the the voltage value from the Digital Battery Capacity Checker and press **Calculate** button


* Arm your quadcopter, and check if all motors are rotating in the direction intended. If no, switch any two wires that are connected to ESC. To arm the drone, put the throttle stick in the bottom right corner. This will start the motors on a quadcopter. To disarm, put the throttle stick in the bottom left corner.


.. danger:: After you install propellers, make sure to keep battery or receiver disconnected while you are working on your quadcopter. Someone may use transmitter bounded to your drone for their own quadcopter as well. The same transmitter can arm several quadcopters!


Driving the rover with the transmitter
------


* Make sure you switch **Kill switch** to off. Select **Manual** as your flight mode.
* Check the battery level, make sure it's enough to perform your first flight.
* Put the quadcopter in the cage and arm. Slowly add throttle while keeping it in the middle of the cage by controlling pitch and yaw.

.. important::
  
  Always check the battery before flying


Odroid installation
------

- Mount Odroid XU4 on the drone

- Solder the UBEC input cable to the power distribution board (or the frame) 

- Solder `Odroid DC Plug Cable <https://www.hardkernel.com/shop/dc-plug-cable-assembly-5-5mm/>`_ to `female servo cable <https://www.sparkfun.com/products/8738>`_ and connect to the UBEC output cable

- In case of MindPX simply connect micro-USB cable to ``USB/OBC`` from the Odroid USB port. In case of Pixhawk use `FTDI module <https://www.ftdichip.com/Support/Documents/DataSheets/Cables/DS_TTL-232R_PCB.pdf>`_. Use `servo cable <https://www.sparkfun.com/products/8738>`_ to solder three wires to GND, TX, and RX. After that solder these three wires to corresponding **TELEM2** port cable. Note that GND connects to GND, RX to TX, and TX to RX.

- Plug in the DC power cable to the Odroid and check if it's powered from the battery


Contributors
-----

Mohammad Albeaik and `Kuat Telegenov <https://github.com/telegek>`_.