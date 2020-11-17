.. _tut-camera:

Retrieve simulation data
========================

.. note:: 
  Currently, all image data are retrieved as CV_8UC3. 
  We are aware of the information loss that occurs and we try to change it ASAP.
  You can also contribute by fixing this on the server- and client-side.


Setup and spawn the camera
--------------------------

.. code-block:: C++

  // Initialization
  std::shared_ptr<Quadrotor> quad_ptr_ = std::make_unique<Quadrotor>();
  std::shared_ptr<RGBCamera> rgb_camera_ = std::make_unique<RGBCamera>();

  // Setup Camera
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_);

  // Setup Quad
  QuadState quad_state_;
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // Spawn
  std::shared_ptr<UnityBridge> unity_bridge_ptr_ = UnityBridge::getInstance();
  unity_bridge_ptr_->addQuadrotor(quad_ptr_);
  bool unity_ready_ = unity_bridge_ptr_->connectUnity(UnityScene::WAREHOUSE);


Position camera
---------------

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


Retrieve data
-------------

.. code-block:: C++

  // Render next frame
  unity_bridge_ptr_->getRender(0);
  unity_bridge_ptr_->handleOutput();

  cv::Mat img;
  rgb_camera_->getRGBImage(img);

  // Save image
  cv::imwrite("some.jpg", img);


[Optional] Publishing data
--------------------------

.. code-block:: C++

  // initialize ROS
  ros::init(argc, argv, "flightmare_rviz");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  image_transport::Publisher rgb_pub_  = it.advertise("/rgb", 1);

  sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp.fromNSec(0);
  
  rgb_pub_.publish(rgb_msg);


Run example
-----------

.. code-block:: bash

  roslaunch flightros camera.launch


.. highlight:: C++ 

Here the full code example
--------------------------

Header
^^^^^^

.. include:: camera.hpp
  :code: C++


Main
^^^^

.. include:: camera.cpp
  :code: C++