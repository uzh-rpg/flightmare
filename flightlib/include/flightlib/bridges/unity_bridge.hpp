#pragma once

// std libs
#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>

// Include ZMQ bindings for communications with Unity.
#include <zmqpp/zmqpp.hpp>

// flightlib
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"

using json = nlohmann::json;

namespace flightlib {

class UnityBridge {
 public:
  // constructor & destructor
  UnityBridge();
  ~UnityBridge(){};

  // connect function
  bool initializeConnections(void);
  bool connectUnity(void);
  bool disconnectUnity(void);

  // public get functions
  bool getRender(const FrameID &frame_id);
  bool handleOutput(RenderMessage_t &output);

  // public set functions
  bool setScene(const SceneID &scene_id);

  //
  bool addQuadrotor(Quadrotor *quad);

  // public auxiliary functions
  inline void setPubPort(const std::string &pub_port) { pub_port_ = pub_port; };
  inline void setSubPort(const std::string &sub_port) { sub_port_ = sub_port; };
  // create unity bridge
  static std::shared_ptr<UnityBridge> getInstance(void) {
    static std::shared_ptr<UnityBridge> bridge_ptr_ =
      std::shared_ptr<UnityBridge>();
    return bridge_ptr_;
  };

 private:
  //
  SettingsMessage_t settings_;
  PubMessage_t pub_msg_;
  Logger logger_{"UnityBridge"};

  std::vector<Quadrotor *> unity_quadrotors_;

  // ZMQ variables and functions
  std::string client_address_;
  std::string pub_port_;
  std::string sub_port_;
  zmqpp::context context_;
  zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
  zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
  bool sendInitialSettings(void);
  bool handleSettings(void);

  // timing variables
  int64_t num_frames_;
  int64_t last_downloaded_utime_;
  int64_t last_download_debug_utime_;
  int64_t u_packet_latency_;

  // connecting symbols
  bool unity_ready_;

  // auxiliary variables
  std::vector<uint8_t> input_buffer_;
};

}  // namespace flightlib