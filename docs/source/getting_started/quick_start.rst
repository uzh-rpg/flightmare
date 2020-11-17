.. _quick-start:

Quick start
===========

Prerequisites
-------------

Git
^^^

For the following installation instructions you will need git which you can set up with the following commands:

.. code-block:: bash

   sudo apt-get install git
   git config --global user.name "Your Name Here"
   git config --global user.email "Same Email as used for github"
   git config --global color.ui true

Packages
^^^^^^^^

Flightmare requires CMake and GCC compiler. You will also need system packages python3, OpenMPI, and OpenCV.

.. code-block:: bash

   apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      libzmqpp-dev \
      libopencv-dev 


Install with pip
-----------------

In this section, we assume that all the prerequisites are installed.

Python environment
^^^^^^^^^^^^^^^^^^

It is a good idea to use virtual environments (virtualenvs) or `Anaconda <https://www.anaconda.com/>`_ to make sure packages from different projects do not interfere with each other. 
Check `here <https://docs.anaconda.com/anaconda/install/linux>`_ for Anaconda installation.

1. To create an environment with python3.6

.. code-block:: bash

   conda create --name ENVNAME python=3.6


2. Activate a named Conda environment

.. code-block:: bash

   conda activate ENVNAME


Install Flightmare
^^^^^^^^^^^^^^^^^^

Clone the project to your desktop (or any other directory)

.. code-block:: bash

   cd ~/Desktop
   git clone https://github.com/uzh-rpg/flightmare.git


Add Environment Variable
^^^^^^^^^^^^^^^^^^^^^^^^

Add **FLIGHTMARE_PATH** environment variable to your `.bashrc` file:

.. code-block:: bash

   echo "export FLIGHTMARE_PATH=~/Desktop/flightmare" >> ~/.bashrc
   source ~/.bashrc


Install dependencies
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   conda activate ENVNAME
   cd flightmare/
   pip install -r requirements.txt


### Install Flighmare (flightlib) 

.. code-block:: bash

   cd flightmare/flightlib
   # it first compile the flightlib and then install it as a python package.
   pip install .


After installing flightlib, you can following the [[Basic Usage with Python|Basic-Usage-with-Python]] for some Reinforcement learning examples. 


Install with ROS
----------------

In this section, we assume that all the prerequisites are installed.

Get ROS
^^^^^^^

You can use this framework with the Robot Operating System `ROS <http://www.ros.org/>`_ and you therefore first need to install it (Desktop-Full Install) by following the steps described in the `ROS Installation <http://wiki.ros.org/ROS/Installation>`_.

Gazebo
""""""

To install Gazebo checkout out their `documentation <http://gazebosim.org/tutorials/?tut=ros_wrapper_versions>`_.

Or in short
* ROS Melodic and newer: use Gazebo version 9.x `sudo apt-get install gazebo9`
* ROS Kinetic and newer: use Gazebo version 7.x `sudo apt-get install gazebo7`
* ROS Indigo: use Gazebo version 2.x `sudo apt-get install gazebo2`

ROS Dependencies
""""""""""""""""

Install system and ROS dependencies (on **Ubuntu20.04**, replace `python-vcstool` with `python3-vcstool` ):

.. code-block:: bash

   sudo apt-get install libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python-vcstool


Before continuing, make sure that your protobuf compiler version is 3.0.0.
To check this out, type in a terminal ``protoc --version``.
If This is not the case, then check out `this guide <https://github.com/linux-on-ibm-z/docs/wiki/Building-ProtoBuf-3.0.0>`_ on how to do it.

Get catkin tools
^^^^^^^^^^^^^^^^^

Get catkin tools with the following commands:

.. code-block:: bash

   sudo apt-get install python-pip 
   sudo pip install catkin-tools


Create a catkin workspace
"""""""""""""""""""""""""

Create a catkin workspace with the following commands:

.. code-block:: bash
   
   cd
   mkdir -p catkin_ws/src
   cd catkin_ws
   catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release


Install Flightmare
^^^^^^^^^^^^^^^^^^

Clone the repository

.. code-block:: bash
   
   cd ~/catkin_ws/src
   git clone https://github.com/uzh-rpg/flightmare.git


Clone dependencies:

.. code-block:: bash
   
   vcs-import < flightmare/flightros/dependencies.yaml


Build:

.. code-block:: bash
   
   catkin build


Add sourcing of your catkin workspace and **FLIGHTMARE_PATH** environment variable to your `.bashrc` file:

.. code-block:: bash

   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   echo "export FLIGHTMARE_PATH=~/catkin_ws/src/flightmare" >> ~/.bashrc
   source ~/.bashrc


Download Flightmare Unity Binary
--------------------------------

Download the Flightmare Unity Binary **RPG_Flightmare.tar.xz** for rendering from the `Releases <https://github.com/uzh-rpg/flightmare/releases>`_ and extract it into the */path/to/flightmare/flightrender*. 

.. raw:: html

    <html>
    <head>
    <style>
    .button {
      background-color: #2980B9;
      border: none;
      color: white;
      padding: 15px 32px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      margin: 4px 2px;
      cursor: pointer;
      transition-duration: 0.4s;
    }
    .button:hover {
      background-color: white; 
      color: #2980B9;
      border: 2px solid #2980B9;
    }
    </style>
    </head>
    <body>
    <center><a href="https://github.com/uzh-rpg/flightmare/releases/latest/download/RPG_Flightmare.tar.xz" class="button">Download</a></center>
    </body>
    </html>

Run Flightmare
--------------

Run pip
^^^^^^^
To use unity rendering, you need first download the binary from **Releases** and extract it into the **flightrender** folder. 
To enable unity for visualization, double click the extracted executable file **RPG_Flightmare.x84-64** and then test a pre-trained controller.

.. code-block:: bash

   conda activate ENVNAME
   cd /path/to/flightmare/flightrl
   pip install .
   cd examples
   python3 run_drone_control.py --train 0 --render 1


Run ROS
^^^^^^^

In this example, we show how to use the `RotorS <https://github.com/ethz-asl/rotors_simulator>`_ for the quadrotor dynamics modelling, `rpg_quadrotor_control <https://github.com/uzh-rpg/rpg_quadrotor_control>`_ for model-based controller, and **Flightmare** for image rendering.

.. code-block:: bash

   # The examples are by default not built.
   catkin build flightros -DBUILD_SAMPLES:=ON

   # Now you can run any example.  
   roslaunch flightros rotors_gazebo.launch


We hope this example can serve as a starting point for many other applications.
For example, Flightmare can be used with other multirotor models that comes with RotorS such as AscTec Hummingbird, the AscTec Pelican, or the AscTec Firefly.
The default controller in `rpg_quadrotor_control <https://github.com/uzh-rpg/rpg_quadrotor_control>`_ is a PID controller. Users have the option to use more advanced controller in this framework, such as `Perception-Aware Model Predictive Control <https://github.com/uzh-rpg/rpg_mpc>`_.

