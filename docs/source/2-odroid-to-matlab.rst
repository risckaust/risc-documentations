ODROID to MATLAB Stream
=====


Intro
-----

Required:

* ODROID with OpenCV installed.

* Computer with OpenCV installed in default locations.

* MATLAB with associated compiler e.g. XCode(Mac OS)/Visual Studio or Microsoft SDK (for Windows)

* WiFi network (Access Point)

* `Streaming ODROID application and MATLAB receiving application <https://github.com/mzahana/Image_Live_Stream>`_.

ODROID setup
-----

Setup OpenCV
^^^^^

Make sure that your odroid is connected to internet.

Open a terminal window, and run the following command,

.. code-block:: bash

	sudo apt-get -y install libopencv-dev

	sudo apt-get -y install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev


Setup streaming app
^^^^

Create a clean directory and navigate to it e.g.

.. code-block:: bash

	cd ~/Desktop
	mkdir imgstream
	cd imgstream

Clone the streaming app from Github

.. code-block:: bash

	git clone https://github.com/mzahana/Image_Live_Stream.git
	cd Image_Live_Stream

Navigate to the ``stream_cpp`` folder, and compile the app

.. code-block:: bash

	cd opencv_stream/stream_cpp
	cmake . & make

If all goes well, then two executable files should be generated: ``sender`` and ``receiver``. Otherwise, make sure that you installed OpenCV properly in the default locations.

To stream images over network, use the ``sender`` app after you connect a camera to ODROID. To use the ``sender`` app, use the following command in a terminal, inside the ``stream_cpp`` folder,

.. code-block:: bash
	
	./sender 192.168.1.100 10000


where ``192.168.1.100`` is the IP of machine running MATLAB (the host machine) (which should be on the same network as the ODROID's). ``10000`` is the port that MATLAB is listening on. Use appropriate IP and port that match the host ones.

MATLAB setup
-----

On MacOS
^^^^^^
Make sure that you installed `XCode <https://developer.apple.com/xcode/>`_ on your Mac OS.

Make sure that you associat your MATLAB with XCode compiler (Google it). Run ``mex -setup`` in MATLAB command line for more information.

Navigate to the ``Image_Live_Stream`` folder that you  downloaded from Github.

Run the ``setup.m`` file

.. code-block:: matlab

	>> setup


If all goes well, you are ready to receive live stream of images from ODROID.

Look at the ``testScript.m`` file to see how you can use the ``ImgStream`` class to establish the connection, and receive image data.

On Windows
^^^^^

Make sure that you install OpenCV 2.4.13 on your Windows. Follow `this video <https://www.youtube.com/watch?v=tHX3MLzwF6Q>`_. It is assumed that you installed the OpenCV folder in ``C:\``

Make sure that your MATLAB is associated with compiler. Run ``mex -setup`` in MATLAB command line for more information.

In MATLAB, run the ``setup.m`` file.

If all goes well, you are ready to receive image stream. Look at the test script to get familiar on how to use the ``ImgStream`` Class.