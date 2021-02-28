.. _server-client:

Server and client
=================


..  source of the image
    https://github.com/uzh-rpg/flightmare/raw/master/docs/flightmare.png

.. image:: ../_images/_getting_started/flightmare_structure.png
    :alt: Simulator structure

The client and the server are two of the fundamentals of Flightmare.  

This tutorial goes from defining the basics and creation of these elements, to describing their possibilities. 

The client
----------

The client is one of the main elements in the Flightmare architecture. 
It connects to the server, retrieves information, and commands changes. 
That is done via scripts, mainly C++. There is also a minimal OpenGym Python wrapper for reinforcement learning tasks. 

Only basic commands will be covered in this section. 
These are useful for things such as spawning lots of actors. 
The rest of the features are more complex, and they will be addressed in their respective pages in :ref:`Advanced steps <advanced-concepts>`.


Connect to server
^^^^^^^^^^^^^^^^^

The server needs to be running.
By default, Flightmare client broadcasts to all IP via ZMQ, and connects to the two ports 10253 and 10254 for publishing respectively subscribing.
The default timeout is 10 seconds.

C++:
""""

.. code-block:: C++

  #include "flightlib/bridges/unity_bridge.hpp"

  std::shared_ptr<UnityBridge> unity_bridge_ptr_ = UnityBridge::getInstance(); 
  bool unity_ready_ = unity_bridge_ptr_->connectUnity(UnityScene::INDUSTRIAL);


Python:
"""""""

.. code-block:: Python

  #!/usr/bin/env python3
  from ruamel.yaml import YAML, dump, RoundTripDumper

  import os

  from rpg_baselines.envs import vec_env_wrapper as wrapper
  from flightgym import QuadrotorEnv_v1

  # select UnityScene in flightmare/flightlib/configs/vec_env.yaml
  cfg = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                            "/flightlib/configs/vec_env.yaml", 'r'))

  env = wrapper.FlightEnvVec(QuadrotorEnv_v1(dump(cfg, Dumper=RoundTripDumper), False))

  env.connectUnity()



Add objects to the server
^^^^^^^^^^^^^^^^^^^^^^^^^

The client has different methods related to quadrotors and objects that allow for different functionalities.  

* Add quadrotors 

.. code-block:: C++

  unity_bridge_ptr_->addQuadrotor(std::shared_ptr<Quadrotor> quad);

* Add sensors

.. code-block:: C++

  unity_bridge_ptr_->addCamera(std::shared_ptr<UnityCamera> unity_camera);

* Add objects

.. code-block:: C++

  unity_bridge_ptr_->addStaticObject(std::shared_ptr<StaticObject> static_object);

More detailed examples can be found on the following pages.

Rendering
^^^^^^^^^

C++
"""
.. code-block:: C++

  unity_bridge_ptr_->getRender(0);
  unity_bridge_ptr_->handleOutput();

Python
""""""

.. code-block:: Python

  env.stepUnity(action, send_id)


The server
----------

The server is the rendering engine of the simulation. 
It runs as the binary or as the top level scene in the Unity editor in play mode. 
It receives messages from the client for the following components:

* Quadrotors and Objects in the simulation

* Sensors

* Environment

* Simulation settings

Debugging
---------
                        

That is a wrap on the server and client. 
The next step takes a closer look into quadrotors and objects to give life to the simulation. 
Keep reading to learn more.