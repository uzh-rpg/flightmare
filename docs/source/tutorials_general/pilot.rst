.. _tut-pilot:

Pilot Tutorial
==============

Simulate the quadrotor with your dynamic model while using Flightmare to generate sensor data.

Use Gazebo dynamics
-------------------

Listen to ROS topic and set state
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: C++

  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
    quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
    quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
    quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
    quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
    quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
    quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
    //
    quad_ptr_->setState(quad_state_);

    if (unity_render_ && unity_ready_) {
      unity_bridge_ptr_->getRender(0);
      unity_bridge_ptr_->handleOutput();
    }
  }
 

Run example
-----------

.. code-block:: bash

  roslaunch flightros rotors_gazebo.launch


.. highlight:: C++ 

Here the full code example
--------------------------

Header
^^^^^^

.. include:: flight_pilot.hpp
  :code: C++


Main
^^^^

.. include:: flight_pilot.cpp
  :code: C++

