.. _sensors-data:

Sensors and data
================

Sensors retrieve data from their surroundings. 
They are crucial to create a learning environment for agents.

This page summarizes everything necessary to start handling sensors. 
It introduces the types available and how to initialize and use them. 

Cameras
-------


This section explains how to spawn, listen to data of camera sensors.

It's assumed that a quadrotor has been initialized as explained on the page :ref:`Quadrotors & Objects <quad-objects>`.

.. list-table:: 
  :widths: 25 25 50
  :header-rows: 1

  * - Sensor
    - Output
    - Overview
  * - RGB
    - CV_8UC3
    - Provides clear vision of the surroundings. Looks like a normal photo of the scene.
  * - Depth
    - CV_8UC3
    - Renders the depth of the elements in the field of view in a gray-scale map.
  * - Segmentation
    - CV_8UC3
    - Renders elements in the field of view with a specific color according to their object type.
  * - Optical flow
    - CV_8UC3
    - Renders the optical flow of the scene.

Check the :ref:`references <cpp-camera-ref>` for all functions.

Spawning
^^^^^^^^

This is how a camera is spawned within Flightmare.
The camera is spawned at a pose relative to the quadrotor.
Post-processing layers like depth, segmentation and optical flow can be enabled.

.. error:: The optical flow is currently incorrect

.. note:: Event camera in development

.. code-block:: C++

  rgb_camera_ = std::make_unique<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_);

Listening
^^^^^^^^^

.. code-block:: C++

    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    cv::Mat img;

    rgb_camera_->getRGBImage(img); // CV_U8C3

    rgb_camera_->getDepthMap(img); // CV_U8C3

    rgb_camera_->getSegmentation(img); // CV_U8C3

    rgb_camera_->getOpticalFlow(img); // CV_U8C3


Publishing
^^^^^^^^^^

.. code-block:: C++

  // initialize ROS
  ros::init(argc, argv, "flightmare_rviz");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);
  
  // initialize publishers
  image_transport::ImageTransport it(pnh);
  rgb_pub_ = it.advertise("/rgb", 1);
  depth_pub_ = it.advertise("/depth", 1);
  segmentation_pub_ = it.advertise("/segmentation", 1);
  opticalflow_pub_ = it.advertise("/opticalflow", 1);

  // ...

  unity_bridge_ptr_->getRender(0);
  unity_bridge_ptr_->handleOutput();

  int frame_id = 0;
  cv::Mat img;

  rgb_camera_->getRGBImage(img);
  sensor_msgs::ImagePtr rgb_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp.fromNSec(frame_id);
  rgb_pub_.publish(rgb_msg);

  rgb_camera_->getDepthMap(img);
  sensor_msgs::ImagePtr depth_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  depth_msg->header.stamp.fromNSec(frame_id);
  depth_pub_.publish(depth_msg);

  rgb_camera_->getSegmentation(img);
  sensor_msgs::ImagePtr segmentation_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  segmentation_msg->header.stamp.fromNSec(frame_id);
  segmentation_pub_.publish(segmentation_msg);

  rgb_camera_->getOpticalFlow(img);
  sensor_msgs::ImagePtr opticflow_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  opticflow_msg->header.stamp.fromNSec(frame_id);
  opticalflow_pub_.publish(opticflow_msg);


Detectors
---------

Collision
^^^^^^^^^

Check if your quadrotor has had a collision in the last simulation step.

.. error:: Not implemented yet

.. code-block:: C++

  bool collision = quad_ptr_->getCollision();


