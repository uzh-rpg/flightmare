#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/logger.hpp"

#include <gtest/gtest.h>

using namespace flightlib;

TEST(UnityBridge, Constructor) {
  Logger logger{"Test Unity Bridge"};
  UnityBridge unity_bridge;
  // unity_bridge.initializeConnections();
  // Scalar time_out_count = 0;
  // Scalar sleep_useconds = 0.2 * 1e5;
  // bool unity_ready = false;
  // logger.info("Trying to Connect Unity.");
  // std::cout << "[";
  // while (!unity_ready) {
  //   // connect unity
  //   unity_ready = unity_bridge.connectUnity();
  //   if (time_out_count / 1e6 > 10) {
  //     std::cout << "]" << std::endl;
  //     logger.warn(
  //       "Unity Connection time out! Make sure that Unity Standalone "
  //       "or Unity Editor is running the Flightmare.");
  //     break;
  //   }
  //   // sleep
  //   usleep(sleep_useconds);
  //   // incread time out counter
  //   time_out_count += sleep_useconds;
  //   std::cout << ".";
  //   std::cout.flush();
  // }
  // if (unity_ready) logger.info("Unity Rendering is connected");
}