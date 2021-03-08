.. _cpp-camera-ref:

Camera References
=================

.. class:: RGBCamera

  .. function:: RGBCamera()

    Construct the RGBCamera.

    :rtype: RGBCamera

  .. function:: ~RGBCamera()

    Deconstruct the RGBCamera.

    :rtype: None


  .. function:: setRelPose(const Ref<Vector<3>> B_r_BC, const Ref<Matrix<3, 3>> R_BC)

   Set the relative position of the camera to quadrotor.

   :param B_r_BC: relative position
   :type B_r_BC: const Ref<Vector<3>>
   :param R_BC: relative orientation
   :type R_BC: const Ref<Matrix<3, 3>>
   :rtype: bool

  .. function:: setWidth(const int width)

   Set the width of the camera screen.

   :param width: screen width
   :type width: const int
   :rtype: bool

  .. function:: setHeight(const int height)

   Set the height of the camera screen.

   :param height: screen height
   :type height: const int
   :rtype: bool

  .. function:: setFOV(const Scalar fov)

    Set the field of view of the camera. 
    Together with the height and the width the camera calibration matrix is defined.

    :param fov: Field of view
    :type fov: const Scalar
    :rtype: bool

  .. function:: setDepthScale(const Scalar depth_scale)

    Set the depth scale. The value needs to be within the range [0.0, 1.0].

    :param depth_scale: Depth scale
    :type depth_scale: const Scalar
    :rtype: bool

  .. function:: setPostProcesscing(const std::vector<bool>& enabled_layers)

    Enable the post-processing layers.

    :param enabled_layers: Layers enables (depth, segmentation, optical flow)
    :type enabled_layers: const std::vector<bool>&
    :rtype: bool

  .. function:: feedImageQueue(const int image_layer, const cv::Mat& image_mat)

    Store OpenCV mat within a queue.

    :param image_layer: Layers (rgb=0, depth=1, segmentation=2, optical flow=3)
    :type image_layer: const int
    :param image_mat: OpenCV Mat (CV_8UC3)
    :type image_mat: const cv::Mat&
    :rtype: bool

  .. 

  .. function:: getEnabledLayers()

    Get a list of enabled layers. (depth, segmentation, optical flow)

    :rtype: std::vector<bool> (Default=false, false, false)

  .. function:: getRelPose()

    Get the relative pose of the camera.

    :rtype:Matrix<4, 4>

  .. function:: getChannels()

    Get the number of channels of the camera.

    :rtype: int (Default=3)

  .. function:: getWidth()

    Get the width of the camera screen.

    :rtype: int (Default=720)

  .. function:: getHeight()

    Get the height of the camera screen.

    :rtype: int (Default=480)

  .. function:: getFOV()

    Get the field of view of the camera.

    :rtype: Scalar (Default=70.0)

  .. function:: getDepthScale()

    Get the depth scale of the camera.

    :rtype: Scalar (Default=0.2)

  .. function:: getRGBImage(cv::Mat& rgb_img)

    Get an image from the RGBImageQueue. Mat format CV_8UC3.

    :rtype: bool, successfully retrieved an image

  .. function:: getDepthMap(cv::Mat& depth_map)

    Get an image from the DepthMapQueue. Mat format CV_8UC3.

    :rtype: bool, successfully retrieved an image

  .. function:: getSegmentation(cv::Mat& segmentation)

    Get an image from the SegmentationQueue. Mat format CV_8UC3.

    :rtype: bool, successfully retrieved an image
  
  .. function:: getOpticalFlow(cv::Mat& opticalflow)

    Get an image from the OpticalFlowQueue. Mat format CV_8UC3.

    :rtype: bool, successfully retrieved an image

  
  .. function::  enableDepth(const bool on)

    Auxiliary function to enable the depth.

    :param on: Enable depth
    :type on: bool
    :rtype: None


  .. function::  enableSegmentation(const bool on)

    Auxiliary function to enable the segmentation.

    :param on: Enable segmentation
    :type on: bool
    :rtype: None

  .. function::  enableOpticalFlow(const bool on)

    Auxiliary function to enable the optical flow.

    :param on: Enable optical flow
    :type on: bool
    :rtype: None

