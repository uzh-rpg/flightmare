.. _introduction:

Flightmare
==========


.. image:: https://github.com/uzh-rpg/flightmare/workflows/CPP_CI/badge.svg
    :target: https://github.com/uzh-rpg/flightmare/actions?query=workflow%3ACPP_CI
    :alt: CPP_CI

.. image:: https://github.com/uzh-rpg/flightmare/workflows/clang_format/badge.svg
    :target: https://github.com/uzh-rpg/flightmare/actions?query=workflow%3Aclang_format
    :alt: clang format

.. image:: https://img.shields.io/badge/License-MIT-blue.svg
   :target: https://github.com/uzh-rpg/flightmare/blob/master/LICENSE
   :alt: License

.. image:: https://img.shields.io/website-up-down-green-red/https/naereen.github.io.svg
   :target: https://uzh-rpg.github.io/flightmare/
   :alt: Website

**Flightmare** is a flexible modular quadrotor simulator.
Flightmare is composed of two main components: 
a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation.
Those two components are totally decoupled and can run independently from each other. 
Flightmare comes with several desirable features: 
(i) a large multi-modal sensor suite, including an interface to extract the 3D point-cloud of the scene; 
(ii) an API for reinforcement learning which can simulate hundreds of quadrotors in parallel; and 
(iii) an integration with a virtual-reality headset for interaction with the simulated environment.
Flightmare can be used for various applications, including path-planning, reinforcement learning, visual-inertial odometry, deep learning, human-robot interaction, etc.

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
    <center><a href="https://uzh-rpg.github.io/flightmare/" class="button">Website</a></center>
    </body>
    </html>



.. raw:: html

    <br>
    <br>
    <center><iframe width="560" height="315" src="https://www.youtube.com/embed/m9Mx1BCNGFU" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe></center>
    <br>
    <br>


The simulator
-------------
Flightmare is an open-source simulator for quadrotors. 
It is composed of two main components: 
a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation. 
In addition, it has OpenAI gym-style python wrapper for reinforcement learning tasks and a flexible interface with stable baselines for solving these tasks with deep RL algorithms. 
Flightmare provides ROS wrapper to interface with popular ROS packages, such as `high_mpc <https://github.com/uzh-rpg/high_mpc>`_ for learning-based mpc, `rpg_mpc <https://github.com/uzh-rpg/rpg_mpc>`_ for advanced quadrotor controller, and `rpg_quadrotor_control <https://github.com/uzh-rpg/rpg_quadrotor_control>`_ for hard-ware-in-the-loop simulation.

..  source of the image
    https://github.com/uzh-rpg/flightmare/raw/master/docs/flightmare.png


.. image:: ../_images/_getting_started/flightmare_structure.png
    :alt: Simulator structure

Software components
^^^^^^^^^^^^^^^^^^^
* `flightlib <https://github.com/uzh-rpg/flightmare/tree/master/flightlib>`_: Flightmare Library

  * Quadrotor Dynamics
  * Sensors Simulation
  * Unity Bridge
  * Python Wrapper
* `flightrender <https://github.com/uzh-rpg/flightmare/tree/master/flightrender>`_: Flightmare Rendering Engine

  * Photo-realistic 3D Environment
  * RGB Images, Depth, Segmentation
* `flightrl <https://github.com/uzh-rpg/flightmare/tree/master/flightrl>`_: Reinforcement Learning Algorithms and Examples

  * Deep Reinforcement Learning Algorithms, e.g., PPO
  * Reinforcement learning examples, e.g., quadrotor control
* `flightros <https://github.com/uzh-rpg/flightmare/tree/master/flightros>`_: ROS Wrapper for Flightmare Library

  * ROS wrapper
  * Quadrotor Control example with PID controller, also simulate RGB Camera that can request images from Flightmare Rendering Engine.



Publication
-----------

.. image:: https://uzh-rpg.github.io/flightmare/assets/paper_thumbnail.png
    :target: https://arxiv.org/abs/2009.00563
    :width: 50%
    :align: center
    :alt: Paper


If you use this code in a publication, please cite the following paper `PDF <https://arxiv.org/abs/2009.00563>`_::
  
  @article{yunlong2020flightmare,
  title={Flightmare: A Flexible Quadrotor Simulator},
  author={Song, Yunlong and Naji, Selim and Kaufmann, Elia and Loquercio, Antonio and Scaramuzza, Davide},
  booktitle={Conference on Robot Learning (CoRL)},
  year={2020}}

  }



License
-------

This project is released under the MIT License. 
Please review the `License file <https://github.com/uzh-rpg/flightmare/blob/master/LICENSE>`_ for more details.


Acknowledgements
----------------

This project is inspired by `FlightGoggles <https://github.com/mit-fast/FlightGoggles>`_, we use some components from FlightGoggles.

The `Image Synthesis for Machine Learning <https://bitbucket.org/Unity-Technologies/ml-imagesynthesis/src/master/>`_ from Unity is a core element of Flightmare's image post-processing.

The demo scene Industrial, which we added in the repository, was created by Dmitrii Kutsenko and is freely available in the asset store. The original asset can be found `here <https://assetstore.unity.com/packages/3d/environments/industrial/rpg-fps-game-assets-for-pc-mobile-industrial-set-v2-0-86679>`_.
