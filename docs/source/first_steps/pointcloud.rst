.. _point-cloud:

Point Cloud
===========

Flightmare can extract the point cloud of a scene.
This data can be used for path planning.

Generate point cloud with flightlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The **unity_bridge** provides a function to save the point cloud.
This sends a ZMQ message from the client to the server. 
The server generates the point cloud and saves it at the specified file path.

.. code-block:: C++

  std::shared_ptr<UnityBridge> unity_bridge_ptr_ = UnityBridge::getInstance(); 
  bool unity_ready_ = unity_bridge_ptr_->connectUnity(UnityScene::INDUSTRIAL);

  PointCloudMessage_t pointcloud_msg;
  pointcloud_msg.path = "/user/";
  pointcloud_msg.file_name = "point_cloud";

  // A point cloud will appear at the path /user/point_cloud.py

.. list-table:: PointCloudMessage_t 
  :widths: 25 25 50
  :header-rows: 1

  * - Parameter
    - Type
    - Overview
  * - range
    - std::vector<Scalar>
    - Defines the range of the extraction area. Default (20,20,20) [m]
  * - origin
    - std::vector<Scalar>
    - Defines the origin of the extraction area. Default (0,0,0)
  * - resolution
    - Scalar
    - Defines the resolution of the point cloud. Default 0.15
  * - path
    - std::string
    - The saving path of the point cloud. Default "pointcloud_data/"
  * - file_name
    - std::string
    - The saving name of the point cloud file in .ply format. Default "default"


Generate point cloud manually
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

One way to generate the point cloud is from the start menu. 
Select the target scene and click on **Scene Save PointCloud**. 
In the now visible point cloud menu, the extraction area can be adjusted and when ready be exported by clicking on **Export PointCloud**. 

The extraction area is the area of interest around the origin in a certain range.
Both can be adjusted with the sliders. 
The extraction area is a slightly transparent red box.
If not visible, you are either within the box or the box is below the ground.
Further also the resolution of the point cloud can be defined.
The **.ply** file can be saved at a specified file path with a specific file name.
If not specified, the point cloud is saved as **default.ply** within the file path pointcloud_data/ of the Flightmare executable.
