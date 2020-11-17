.. _quad-objects:

Quadrotors and objects
======================

This page introduces all dynamically spawn-able actors within Flightmare.

Unity Prefabs
-------------

Prefabs are already-made Unity GameObjects with animations and a series of attributes. 
These attributes include, among others, mesh, material, post-processing settings, and much more.

Quadrotors
----------

In this section, we explain how you can spawn and move the quadrotor. 
Check the :ref:`references <cpp-quad-ref>` for all functions.

Spawning
^^^^^^^^

To spawn the quadrotor in a scene, you have to implement the following lines of code.

.. code-block:: C++

  #include "flightlib/bridges/unity_bridge.hpp"
  #include "flightlib/common/quad_state.hpp"
  #include "flightlib/common/types.hpp"
  #include "flightlib/objects/quadrotor.hpp"

  using namespace flightlib;
  
  // Initialize quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_ = std::make_shared<Quadrotor>();
  QuadState quad_state_;
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // Initialize Unity bridge
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  unity_bridge_ptr_ = UnityBridge::getInstance();

  // Add quadrotor
  unity_bridge_ptr_->addQuadrotor(quad_ptr_);
  bool unity_ready_ = unity_bridge_ptr_->connectUnity(UnityScene::WAREHOUSE);


Set State
^^^^^^^^^

When the quadrotor is spawned, the pose can be updated by the following lines of code.

.. code-block:: C++

  // Define new quadrotor state
  quad_state_.x[QS::POSX] = (Scalar)position.x;
  quad_state_.x[QS::POSY] = (Scalar)position.y;
  quad_state_.x[QS::POSZ] = (Scalar)position.z;
  quad_state_.x[QS::ATTW] = (Scalar)orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)orientation.z;
  
  // Set new state
  quad_ptr_->setState(quad_state_);

  // Render next frame
  unity_bridge_ptr_->getRender(0);
  unity_bridge_ptr_->handleOutput();


Gates
-----

Objects can be dynamically placed within a static scene.
The class StaticGate inherit from the class StaticObject, so also other static objects can be added to the scene.
The model loads the prefab matching the prefab_id in the folder Assets/Resources. 
Check the :ref:`references <cpp-gate-ref>` for all functions.

Spawning
^^^^^^^^

To spawn a gate in a scene, you have to implement the following lines of code.

.. code-block:: C++

  #include <Eigen/Dense>
  
  #include "flightlib/bridges/unity_bridge.hpp"
  #include "flightlib/bridges/unity_message_types.hpp"
  #include "flightlib/common/types.hpp"
  #include "flightlib/objects/static_gate.hpp"

  using namespace flightlib;

  // Initialize gates
  std::string object_id = "unity_gate"; // Unique name
  std::string prefab_id = "rpg_gate"; // Name of the prefab in the Assets/Resources folder
  std::shared_ptr<StaticGate> gate =
    std::make_shared<StaticGate>(object_id, prefab_id);
  gate->setPosition(Eigen::Vector3f(0, 10, 2.5));
  gate->setRotation(
    Quaternion(std::cos(0.5 * M_PI_2), 0.0, 0.0, std::sin(0.5 * M_PI_2)));

  // Initialize Unity bridge
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  unity_bridge_ptr_ = UnityBridge::getInstance();
  
  // Add gates
  unity_bridge_ptr_->addStaticObject(gate);
  bool unity_ready_ = unity_bridge_ptr_->connectUnity(UnityScene::WAREHOUSE);


