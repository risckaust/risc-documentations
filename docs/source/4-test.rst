PX4 Pro Drone Autopilot
=======================

`Releases <https://github.com/PX4/Firmware/releases>`__
`DOI <https://zenodo.org/badge/latestdoi/22634/PX4/Firmware>`__

`Build
Status <http://ci.px4.io:8080/blue/organizations/jenkins/Firmware/activity>`__
`Coverity Scan <https://scan.coverity.com/projects/3966?tab=overview>`__

`Slack <http://slack.px4.io>`__

This repository holds the `PX4 Pro <http://px4.io>`__ flight control
solution for drones, with the main applications located in the
`src/modules <https://github.com/PX4/Firmware/tree/master/src/modules>`__
directory. It also contains the PX4 Drone Middleware Platform, which
provides drivers and middleware to run drones.

-  Official Website: http://px4.io (License: BSD 3-clause,
   `LICENSE <https://github.com/PX4/Firmware/blob/master/LICENSE>`__)
-  `Supported
   airframes <https://docs.px4.io/en/airframes/airframe_reference.html>`__
   (`portfolio <http://px4.io/#airframes>`__):

   -  `Multicopters <https://docs.px4.io/en/airframes/airframe_reference.html#copter>`__
   -  `Fixed
      wing <https://docs.px4.io/en/airframes/airframe_reference.html#plane>`__
   -  `VTOL <https://docs.px4.io/en/airframes/airframe_reference.html#vtol>`__
   -  many more experimental types (Rovers, Blimps, Boats, Submarines,
      etc)

-  Releases: `Downloads <https://github.com/PX4/Firmware/releases>`__

PX4 Users
---------

The `PX4 User Guide <https://docs.px4.io/en/>`__ explains how to
assemble `supported
vehicles <https://docs.px4.io/en/airframes/airframe_reference.html>`__
and fly drones with PX4. See the `forum and
chat <https://docs.px4.io/en/#support>`__ if you need help!

PX4 Developers
--------------

This `Developer Guide <https://dev.px4.io/>`__ is for software
developers who want to modify the flight stack and middleware (e.g. to
add new flight modes), hardware integrators who want to support new
flight controller boards and peripherals, and anyone who wants to get
PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the `Guide for
Contributions <https://dev.px4.io/en/contribute/>`__. See the `forum and
chat <https://dev.px4.io/en/#support>`__ if you need help!

Weekly Dev Call
~~~~~~~~~~~~~~~

The PX4 Dev Team syncs up on a `weekly dev
call <https://dev.px4.io/en/contribute/#dev_call>`__.

   **Note** The dev call is open to all interested developers (not just
   the core dev team). This is a great opportunity to meet the team and
   contribute to the ongoing development of the platform.

Maintenance Team
----------------

-  Project / Founder - `Lorenz Meier <https://github.com/LorenzMeier>`__

   -  `Dev Call <https://github.com/PX4/Firmware/labels/devcall>`__ -
      `Ramon Roche <https://github.com/mrpollo>`__

-  Communication Architecture

   -  `Beat Kueng <https://github.com/bkueng>`__
   -  `Julian Oes <https://github.com/JulianOes>`__

-  UI / UX

   -  `Donald Gagne <https://github.com/DonLakeFlyer>`__
   -  `Gus Grubba <https://github.com/dogmaphobic>`__

-  `Multicopter Flight
   Control <https://github.com/PX4/Firmware/labels/multicopter>`__

   -  `Dennis Mannhart <https://github.com/Stifael>`__
   -  `Matthias Grob <https://github.com/MaEtUgR>`__

-  `VTOL Flight Control <https://github.com/PX4/Firmware/labels/vtol>`__

   -  `Daniel Agar <https://github.com/dagar>`__
   -  `Mathieu Bresciani <https://github.com/bresch>`__
   -  `Sander Smeets <https://github.com/sanderux>`__
   -  `Roman Bapst <https://github.com/tumbili>`__
   -  `Andreas Antener <https://github.com/AndreasAntener>`__

-  `Fixed Wing Flight
   Control <https://github.com/PX4/Firmware/labels/fixedwing>`__

   -  `Daniel Agar <https://github.com/dagar>`__
   -  `Paul Riseborough <https://github.com/priseborough>`__

-  Racers - `Matthias Grob <https://github.com/MaEtUgR>`__
-  OS / drivers - `David Sidrane <https://github.com/davids5>`__
-  `UAVCAN <https://github.com/PX4/Firmware/labels/uavcan>`__ /
   Industrial - `Pavel Kirienko <https://github.com/pavel-kirienko>`__
-  `State
   Estimation <https://github.com/PX4/Firmware/issues?q=is%3Aopen+is%3Aissue+label%3A%22state+estimation%22>`__
   - `James Goppert <https://github.com/jgoppert>`__, `Paul
   Riseborough <https://github.com/priseborough>`__
-  Vision based navigation

   -  `Christoph Tobler <https://github.com/ChristophTobler>`__
   -  `Mohammed Kabir <https://github.com/mhkabir>`__

-  Obstacle Avoidance - `Vilhjalmur
   Vilhjalmsson <https://github.com/vilhjalmur89>`__
-  `Snapdragon <https://github.com/PX4/Firmware/labels/snapdragon>`__

   -  `Christoph Tobler <https://github.com/ChristophTobler>`__
   -  `Mark Charlebois <https://github.com/mcharleb>`__

-  `Intel Aero <https://github.com/PX4/Firmware/labels/intel%20aero>`__

   -  `Sugnan Prabhu <https://github.com/sugnanprabhu>`__
   -  `José Roberto de Souza <https://github.com/zehortigoza>`__

-  `Raspberry Pi /
   Navio <https://github.com/PX4/Firmware/labels/raspberry_pi>`__ -
   `Beat Kueng <https://github.com/bkueng>`__
-  `Parrot Bebop <https://github.com/PX4/Firmware/labels/bebop>`__ -
   `Michael Schaeuble <https://github.com/eyeam3>`__
-  `Airmind MindPX /
   MindRacer <https://github.com/PX4/Firmware/labels/mindpx>`__ - `Henry
   Zhang <https://github.com/iZhangHui>`__
-  RTPS/ROS2 Interface - `Vicente
   Monge <https://github.com/vicenteeprosima>`__

See also `About Us <http://px4.io/about-us/#development_team>`__
(px4.io) and the `contributors
list <https://github.com/PX4/Firmware/graphs/contributors>`__ (Github).

Supported Hardware
------------------

This repository contains code supporting these boards: \* `Snapdragon
Flight <https://docs.px4.io/en/flight_controller/snapdragon_flight.html>`__
\* `Intel
Aero <https://docs.px4.io/en/flight_controller/intel_aero.html>`__ \*
`Raspberry PI with Navio
2 <https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html>`__
\* `Parrot Bebop 2 <https://dev.px4.io/en/advanced/parrot_bebop.html>`__
\* FMUv2.x \*
`Pixhawk <https://docs.px4.io/en/flight_controller/pixhawk.html>`__ \*
`Pixhawk
Mini <https://docs.px4.io/en/flight_controller/pixhawk_mini.html>`__ \*
`Pixfalcon <https://docs.px4.io/en/flight_controller/pixfalcon.html>`__
\* FMUv3.x `Pixhawk 2 <https://pixhawk.org/modules/pixhawk2>`__ \*
FMUv4.x \*
`Pixracer <https://docs.px4.io/en/flight_controller/pixracer.html>`__ \*
`Pixhawk 3
Pro <https://docs.px4.io/en/flight_controller/pixhawk3_pro.html>`__ \*
FMUv5.x (ARM Cortex M7, future Pixhawk) \*
`STM32F4Discovery <http://www.st.com/en/evaluation-tools/stm32f4discovery.html>`__
(basic support)
`Tutorial <https://pixhawk.org/modules/stm32f4discovery>`__ \* `Gumstix
AeroCore <https://www.gumstix.com/aerocore-2/>`__ (only v2) \* `Airmind
MindPX
V2.8 <http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf>`__
\* `Airmind MindRacer
V1.2 <http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf>`__
\* `Bitcraze Crazyflie
2.0 <https://docs.px4.io/en/flight_controller/crazyflie2.html>`__

Additional information about supported hardware can be found in `PX4
user Guide > Autopilot
Hardware <https://docs.px4.io/en/flight_controller/>`__.

Project Roadmap
---------------

A high level project roadmap is available
`here <https://www.dronecode.org/roadmap/>`__.
