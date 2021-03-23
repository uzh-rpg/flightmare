.. _tut-racing:

Navigation through waypoints
============================

In this section, we show how to use `rpg_quadrotor_control <https://github.com/uzh-rpg/rpg_quadrotor_control>`_ with Flightmare to navigate through waypoints.

Trajectory
^^^^^^^^^^

Generate trajectory
"""""""""""""""""""

.. code-block:: C++

  // Define path through gates
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(0, 10, 2.5));
  way_points.push_back(Eigen::Vector3d(5, 0, 2.5));
  way_points.push_back(Eigen::Vector3d(0, -10, 2.5));
  way_points.push_back(Eigen::Vector3d(-5, 0, 2.5));

  std::size_t num_waypoints = way_points.size();
  Eigen::VectorXd segment_times(num_waypoints);
  segment_times << 10.0, 10.0, 10.0, 10.0;
  Eigen::VectorXd minimization_weights(5);
  minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;

  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
    polynomial_trajectories::PolynomialTrajectorySettings(
      way_points, minimization_weights, 7, 4);

  polynomial_trajectories::PolynomialTrajectory trajectory =
    polynomial_trajectories::minimum_snap_trajectories::
      generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,
                                        20.0, 20.0, 6.0);


Get point from trajectory
"""""""""""""""""""""""""

.. code-block:: C++

  manual_timer timer;
  timer.start();

  while (ros::ok()) {
    timer.stop();

    quadrotor_common::TrajectoryPoint desired_pose =
      polynomial_trajectories::getPointFromTrajectory(
        trajectory, ros::Duration(timer.get() / 1000));

    // Set pose
    quad_state_.x[QS::POSX] = (Scalar)desired_pose.position.x();
    quad_state_.x[QS::POSY] = (Scalar)desired_pose.position.y();
    quad_state_.x[QS::POSZ] = (Scalar)desired_pose.position.z();
    quad_state_.x[QS::ATTW] = (Scalar)desired_pose.orientation.w();
    quad_state_.x[QS::ATTX] = (Scalar)desired_pose.orientation.x();
    quad_state_.x[QS::ATTY] = (Scalar)desired_pose.orientation.y();
    quad_state_.x[QS::ATTZ] = (Scalar)desired_pose.orientation.z();

    quad_ptr_->setState(quad_state_);

    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();
  }


Run example
-----------

.. code-block:: bash

  roslaunch flightros racing.launch



.. highlight:: C++ 

Here the full code example
--------------------------

Header
^^^^^^

.. include:: racing.hpp
  :code: C++


Main
^^^^

.. include:: racing.cpp
  :code: C++

