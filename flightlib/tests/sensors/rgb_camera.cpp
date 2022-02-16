

#include "flightlib/sensors/rgb_camera.hpp"

#include <gtest/gtest.h>

using namespace flightlib;

TEST(RGBCamera, Constructor) {
  RGBCamera camera = RGBCamera();

  const int init_width = 640;
  const int init_height = 480;
  const int init_channels = 3;
  const Scalar init_fov = 90.0;

  std::vector<bool> init_enabled_layers;
  init_enabled_layers.push_back(true);
  init_enabled_layers.push_back(true);
  init_enabled_layers.push_back(true);

  EXPECT_TRUE(camera.setWidth(init_width));
  EXPECT_TRUE(camera.setHeight(init_height));
  EXPECT_TRUE(camera.setChannels(init_channels));
  EXPECT_TRUE(camera.setFOV(init_fov));
  EXPECT_TRUE(camera.setPostProcessing(init_enabled_layers));

  const int width = camera.getWidth();
  const int height = camera.getHeight();
  const int channels = camera.getChannels();
  const Scalar fov = camera.getFOV();

  EXPECT_EQ(init_height, height);
  EXPECT_EQ(init_width, width);
  EXPECT_EQ(init_fov, fov);
  EXPECT_EQ(init_channels, channels);
}