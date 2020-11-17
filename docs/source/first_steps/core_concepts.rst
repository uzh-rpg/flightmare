.. _core-concepts:

Core concepts
=============

This page introduces the main features and modules in Flightmare. 
Detailed explanations of the different subjects can be found on their corresponding page.

:ref:`Server and Client <server-client>`
----------------------------------------

..  source of the image
    https://github.com/uzh-rpg/flightmare/raw/master/docs/flightmare.png

.. image:: ../_images/_getting_started/flightmare_structure.png
    :alt: Simulator structure

The client is the module the user runs to model the dynamics in the simulation. 
A client runs with an IP and a specific port. 
It communicates with the server via terminal. 

The server is the rendering engine representing the simulation. 
It contains the main methods to spawn quadrotors, change the environment, get the current state of the environment, etc.

:ref:`Quadrotor and Objects <quad-objects>`
-------------------------------------------

An actor is anything that plays a role in the simulation and is loaded dynamically into the environment.

* Quadrotor

* Sensors

* Gates

The prefabs of the actor can be found in *Assets/Resources/* on the server-side.
They can be added by the client over the unity-bridge.



:ref:`Environments and navigation <environments-navigation>`
------------------------------------------------------------

The environment is the object representing the simulated world. 
There are four environments available in the binary. 

.. include:: ../building_flightmare_binary/faq.rst
    :start-after: The binaries is released with three environments.
    :end-before: The git repository version only includes the environment

Custom environments can be added (see :ref:`here <add-scene>`).

.. include:: ../building_flightmare_binary/faq.rst
    :start-after: Yes, follow the steps :ref:`here <add-scene>`.
    :end-before: Why are some objects flickering?

For navigation we use the library: `rpg_quadrotor_control <https://github.com/uzh-rpg/rpg_quadrotor_control>`_.


:ref:`Sensors and data <sensors-data>`
--------------------------------------

Sensors wait for rendering to happen, and then gather data from the simulation. 
They call for a function defining how to handle the data. 
Depending on which, sensors retrieve different types of sensor data.

A sensor is an actor attached to a parent quadrotor. 
It follows the quadrotor around, gathering information about the surroundings. 
The following sensors are available:

* Cameras 

  * RGB

  * Depth 

  * Semantic segmentation

* Collision detector

* Soon to be added

  * IMU sensor

  * Lidar raycast

  * Optical flow camera

  * Event-based camera


:ref:`Point Cloud <point-cloud>`
--------------------------------

Flightmare can extract the point cloud of a scene.
This data can be used for path planning.
