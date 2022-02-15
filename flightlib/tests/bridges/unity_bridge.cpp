#include "flightlib/bridges/unity_bridge.hpp"

#include <gtest/gtest.h>

#include "flightlib/common/logger.hpp"
#include "flightlib/objects/static_gate.hpp"

using namespace flightlib;

TEST(UnityBridge, Constructor) {
  Logger logger{"Test Unity Bridge"};
  UnityBridge unity_bridge;
  bool unity_ready = false;

  // need to add a quad to connect to Flightmare
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  unity_bridge.addQuadrotor(quad);

  unity_ready = unity_bridge.connectUnity(UnityScene::GARAGE);

  if (unity_ready) logger.info("Unity Rendering is connected");
  EXPECT_TRUE(unity_ready);
  // timeout flightmare
  usleep(5 * 1e6);
}

TEST(UnityBridge, PointCloud) {
  Logger logger{"Test PointCloud"};
  UnityBridge unity_bridge;

  // need to add a quad to connect to Flightmare
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  unity_bridge.addQuadrotor(quad);

  EXPECT_TRUE(unity_bridge.connectUnity(UnityScene::GARAGE));
  PointCloudMessage_t pointcloud_msg;
  pointcloud_msg.path = "/tmp/";
  pointcloud_msg.file_name = "unity-bridge" + std::to_string(::rand());
  EXPECT_TRUE(unity_bridge.getPointCloud(pointcloud_msg, 30.0));
  std::filesystem::remove(pointcloud_msg.path + pointcloud_msg.file_name +
                          ".ply");
  // timeout flightmare
  usleep(5 * 1e6);
}

TEST(UnityBridge, HandleOutputRGB) {
  Logger logger{"Test HandleOutputRGB"};
  UnityBridge unity_bridge;
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  std::shared_ptr<RGBCamera> rgb = std::make_shared<RGBCamera>();
  quad->addRGBCamera(rgb);
  unity_bridge.addQuadrotor(quad);

  EXPECT_TRUE(unity_bridge.connectUnity(UnityScene::GARAGE));

  FrameID send_frame_id = 1;
  unity_bridge.getRender(send_frame_id);
  FrameID received_frame_id = unity_bridge.handleOutput(send_frame_id);

  EXPECT_EQ(send_frame_id, received_frame_id);

  cv::Mat test_img;

  EXPECT_TRUE(rgb->getRGBImage(test_img));
  EXPECT_FALSE(rgb->getDepthMap(test_img));
  EXPECT_FALSE(rgb->getSegmentation(test_img));
  EXPECT_FALSE(rgb->getOpticalFlow(test_img));

  // timeout flightmare
  usleep(5 * 1e6);
}

TEST(UnityBridge, HandleOutputDepth) {
  Logger logger{"Test HandleOutputDepth"};
  UnityBridge unity_bridge;
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  std::shared_ptr<RGBCamera> rgb = std::make_shared<RGBCamera>();
  rgb->setPostProcessing(std::vector<bool>{true, false, false});
  quad->addRGBCamera(rgb);
  unity_bridge.addQuadrotor(quad);

  EXPECT_TRUE(unity_bridge.connectUnity(UnityScene::GARAGE));

  FrameID send_frame_id = 1;
  unity_bridge.getRender(send_frame_id);
  FrameID received_frame_id = unity_bridge.handleOutput(send_frame_id);


  EXPECT_EQ(send_frame_id, received_frame_id);

  cv::Mat test_img;

  EXPECT_TRUE(rgb->getRGBImage(test_img));
  EXPECT_TRUE(rgb->getDepthMap(test_img));
  EXPECT_FALSE(rgb->getSegmentation(test_img));
  EXPECT_FALSE(rgb->getOpticalFlow(test_img));

  // timeout flightmare
  usleep(5 * 1e6);
}

TEST(UnityBridge, HandleOutputSegmentation) {
  Logger logger{"Test HandleOutputSegmentation"};
  UnityBridge unity_bridge;
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  std::shared_ptr<RGBCamera> rgb = std::make_shared<RGBCamera>();
  rgb->setPostProcessing(std::vector<bool>{false, true, false});
  quad->addRGBCamera(rgb);
  unity_bridge.addQuadrotor(quad);

  EXPECT_TRUE(unity_bridge.connectUnity(UnityScene::GARAGE));

  FrameID send_frame_id = 1;
  unity_bridge.getRender(send_frame_id);
  FrameID received_frame_id = unity_bridge.handleOutput(send_frame_id);

  EXPECT_EQ(send_frame_id, received_frame_id);

  cv::Mat test_img;

  EXPECT_TRUE(rgb->getRGBImage(test_img));
  EXPECT_FALSE(rgb->getDepthMap(test_img));
  EXPECT_TRUE(rgb->getSegmentation(test_img));
  EXPECT_FALSE(rgb->getOpticalFlow(test_img));

  // timeout flightmare
  usleep(5 * 1e6);
}

TEST(UnityBridge, HandleOutputOpticalFlow) {
  Logger logger{"Test HandleOutputOpticalFlow"};
  UnityBridge unity_bridge;
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  std::shared_ptr<RGBCamera> rgb = std::make_shared<RGBCamera>();
  rgb->setPostProcessing(std::vector<bool>{false, false, true});
  quad->addRGBCamera(rgb);
  unity_bridge.addQuadrotor(quad);

  EXPECT_TRUE(unity_bridge.connectUnity(UnityScene::GARAGE));

  FrameID send_frame_id = 1;
  unity_bridge.getRender(send_frame_id);
  FrameID received_frame_id = unity_bridge.handleOutput(send_frame_id);

  EXPECT_EQ(send_frame_id, received_frame_id);

  cv::Mat test_img;

  EXPECT_TRUE(rgb->getRGBImage(test_img));
  EXPECT_FALSE(rgb->getDepthMap(test_img));
  EXPECT_FALSE(rgb->getSegmentation(test_img));
  EXPECT_TRUE(rgb->getOpticalFlow(test_img));
  cv::imwrite("/tmp/optical.png", test_img);

  // timeout flightmare
  usleep(5 * 1e6);
}

TEST(UnityBridge, SpawnStaticGate) {
  Logger logger{"Test SpawnStaticGate"};
  UnityBridge unity_bridge;
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  flightlib::QuadState state;
  state.setZero();
  state.x[QS::POSX] = (Scalar)0;
  state.x[QS::POSY] = (Scalar)-10;
  state.x[QS::POSZ] = (Scalar)-3;
  quad->setState(state);
  std::shared_ptr<RGBCamera> rgb = std::make_shared<RGBCamera>();
  rgb->setPostProcessing(std::vector<bool>{false, false, true});
  quad->addRGBCamera(rgb);
  unity_bridge.addQuadrotor(quad);

  std::string object_id = "unity_gate";
  std::shared_ptr<StaticGate> obj = std::make_shared<StaticGate>(object_id);
  obj->setPosition(Eigen::Vector3d(0, -10, -3));
  unity_bridge.addStaticObject(obj);

  EXPECT_TRUE(unity_bridge.connectUnity(UnityScene::GARAGE));
  // timeout flightmare
  usleep(5 * 1e6);
}

TEST(UnityBridge, Spawn100StaticGate) {
  Logger logger{"Test Spawn100StaticGate"};
  UnityBridge unity_bridge;
  QuadrotorDynamics dyn = QuadrotorDynamics(1.0);
  std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  flightlib::QuadState state;
  state.setZero();
  state.x[QS::POSX] = (Scalar)0;
  state.x[QS::POSY] = (Scalar)-10;
  state.x[QS::POSZ] = (Scalar)-3;
  quad->setState(state);
  std::shared_ptr<RGBCamera> rgb = std::make_shared<RGBCamera>();
  rgb->setPostProcessing(std::vector<bool>{false, false, true});
  quad->addRGBCamera(rgb);
  unity_bridge.addQuadrotor(quad);


  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 25; j++) {
      std::string object_id = "unity_gate" + std::to_string(25 * i + j);
      std::shared_ptr<StaticGate> obj = std::make_shared<StaticGate>(object_id);
      obj->setPosition(Eigen::Vector3d(j - 12, -10 + i, -3));
      unity_bridge.addStaticObject(obj);
    }
  }

  EXPECT_TRUE(unity_bridge.connectUnity(UnityScene::GARAGE));
  // timeout flightmare
  usleep(5 * 1e6);
}
