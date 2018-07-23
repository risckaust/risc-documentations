Multi-point Telemetry
=====================

In this tutorial, the objective is to make the setup which allows to communicate with multiple telemetry modules using a single base telemetry module. The use case is a single telemetry module (e.g. 3DR or RFD900) is connected to ground station that runs QGroundControl that monitors/controls a fleet of drones. Each drone has a single telemetry module. So, it's one-to-many network.

**We will need the** `SiK_Multipoint <https://github.com/RFDesign/SiK/tree/SiK_Multipoint/>`_.



Installation
------------

Install required packages.

On Mac,

.. code-block:: bash

  brew install sdcc

On Ubuntu,

.. code-block:: bash

  sudo apt update; sudo apt install sdcc


Clone the SiK package and switch to branch

.. code-block:: bash

  cd ~
  mkdir ~/src
  cd ~/src
  git clone https://github.com/RFDesign/SiK.git
  cd SiK
  git checkout SiK_Multipoint


Make and install,

.. code-block:: bash

  cd SiK/Firmware
  make install

Upload Firmware to the radio
-----------------------------

**Change the serial port name**

.. code-block:: bash

  tools/uploader.py --port /dev/tty.usbserial-CHANGETHIS dst/radio~hm_trp.ihx

.. note::

  If you get errors, try to update `pyserial` module



Device Configuration
--------------------

Start command mode,

.. code-block:: bash

  screen /dev/tty.usbserial-CHANGETHIS 57600 8N1

then type,

.. code-block:: bash

  +++

**Wait one second before you type anything**

To list all editable parameters type,

.. code-block:: bash

  ATI5

To change a paramter use,

.. code-block:: bash

  ATS<parameternumber>=<value>

Make sure you save by typing,

.. code-block:: bash

  AT&W

.. note::

* Set ``MAVLINK=1``
* Set ``NODECOUNT`` to the number of used telemetry modules
* There must be a base module with ``NODEID=0``
* Put base node in broadcast mode by setting ``NODEDESTINATION=65535``
* All other nodes should to talk to base only by setting ``NODEDESTINATION=0``

.. warning::

  Make sure that you save parameters after each set using ``AT&W``. Otherwise, parameters changes won't survive restes.

References
-----------

* `<https://github.com/RFDesign/SiK/tree/SiK_Multipoint>`_

* `<http://dev.px4.io/en/data_links/sik_radio.html>`_



