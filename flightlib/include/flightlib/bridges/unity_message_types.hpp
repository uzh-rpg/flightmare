//
// This bridge message types was originally from FlightGoggles.
// We made several changes on top of it.
//
#pragma once

// std
#include <string>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// flightlib
#include "flightlib/common/types.hpp"
#include "flightlib/json/json.hpp"

using json = nlohmann::json;

namespace flightlib {

enum UnityScene {
  WAREHOUSE = 0,
  GARAGE = 1,
  TUNELS = 2,
  NATUREFOREST = 3,
  // total number of environment
  JAPAN = 4,
  SceneNum = 5
};


struct EventsMessage_t {
  std::vector<Event_t> events;
};
// Unity Camera, should not be used alone.
// has to be attached on a vehicle.
struct Camera_t {
  std::string ID;
  // frame Metadata
  int channels{3};
  int width{1024};
  int height{768};
  Scalar fov{70.0f};
  Scalar depth_scale{0.20};  // 0.xx corresponds to xx cm resolution
  // metadata
  bool is_depth{false};
  int output_index{0};
  //
  std::vector<bool> enabled_layers;
  // Transformation matrix from camera to vehicle body 4 x 4
  // use 1-D vector for json convention
  std::vector<Scalar> T_BC{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct EventCamera_t {
  std::string ID;
  // frame Metadata
  int channels{3};
  int width{346};
  int height{260};
  Scalar fov{70.0f};
  Scalar depth_scale{0.20};  // 0.xx corresponds to xx cm resolution
  // metadata
  bool is_depth{false};
  int output_index{0};
  // eventcamera settings
  float Cm{0.1};
  float Cp{0.1};
  float sigma_Cp{0};
  float sigma_Cm{0};
  uint64_t refractory_period_ns{0};
  float log_eps{0.0001};
  // Transformation matrix from camera to vehicle body 4 x 4
  // use 1-D vector for json convention
  std::vector<Scalar> T_BC{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct Lidar_t {
  std::string ID;
  int num_beams{10};
  Scalar max_distance{10.0};
  Scalar start_scan_angle{-M_PI / 2};
  Scalar end_scan_angle{M_PI / 2};
  // Transformation matrix from lidar to vehicle body 4 x 4
  // use 1-D vector for json convention
  std::vector<Scalar> T_BS{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

// Unity Vechicle, e.g., quadrotor
struct Vehicle_t {
  std::string ID;
  // unity coordinate system left-handed, y up
  std::vector<Scalar> position{0.0, 0.0, 0.0};
  // unity quaternion (x, y, z, w)
  std::vector<Scalar> rotation{0.0, 0.0, 0.0, 1.0};
  std::vector<Scalar> size{1.0, 1.0, 1.0};  // scale
  // sensors attached on the vehicle
  std::vector<Camera_t> cameras;
  std::vector<EventCamera_t> eventcameras;
  std::vector<Lidar_t> lidars;
  // collision check
  bool has_collision_check = true;
};

// Unity object, e.g., gate, light, etc...
struct Object_t {
  std::string ID;
  std::string prefab_ID;
  // unity coordinate system left hand, y up
  std::vector<Scalar> position{0.0, 0.0, 0.0};
  // unity quaternion (x, y, z, w)
  std::vector<Scalar> rotation{0.0, 0.0, 0.0, 1.0};
  std::vector<Scalar> size{1.0, 1.0, 1.0};  // scale
};

struct SettingsMessage_t {
  // scene/render settings
  size_t scene_id = UnityScene::WAREHOUSE;
  std::vector<Vehicle_t> vehicles;
  std::vector<Object_t> objects;
};

struct PubMessage_t {
  FrameID frame_id{0};
  std::vector<Vehicle_t> vehicles;
  std::vector<Object_t> objects;
};

//
struct Sub_Vehicle_t {
  bool collision;
  std::vector<Scalar> lidar_ranges;
};

struct SubMessage_t {
  FrameID frame_id{0};
  std::vector<Sub_Vehicle_t> sub_vehicles;
};

struct PointCloudMessage_t {
  // define point cloud box range [x, y, z] / meter
  std::vector<Scalar> range{20.0, 20.0, 20.0};
  std::vector<Scalar> origin{0.0, 0.0, 0.0};
  Scalar resolution{0.15};
  std::string path{"point_clouds_data/"};
  std::string file_name{"default"};
};

/*********************
 * JSON constructors *
 *********************/
// Camera_t
inline void to_json(json &j, const Camera_t &o) {
  j = json{{"ID", o.ID},
           {"channels", o.channels},
           {"width", o.width},
           {"height", o.height},
           {"fov", o.fov},
           {"T_BC", o.T_BC},
           {"isDepth", o.is_depth},
           {"enabledLayers", o.enabled_layers},
           {"depthScale", o.depth_scale},
           {"outputIndex", o.output_index}};
}
// EventCamera_t
inline void to_json(json &j, const EventCamera_t &o) {
  j = json{{"ID", o.ID},
           {"channels", o.channels},
           {"width", o.width},
           {"height", o.height},
           {"fov", o.fov},
           {"T_BC", o.T_BC},
           {"isDepth", o.is_depth},
           {"depthScale", o.depth_scale},
           {"outputIndex", o.output_index},
           {"Cm", o.Cm},
           {"Cp", o.Cp},
           {"sigma_Cp", o.sigma_Cp},
           {"sigma_Cm", o.sigma_Cm},
           {"refractory_period_ns", o.refractory_period_ns},
           {"log_eps", o.log_eps}};
}

// Lidar
inline void to_json(json &j, const Lidar_t &o) {
  j = json{{"ID", o.ID},
           {"num_beams", o.num_beams},
           {"max_distance", o.max_distance},
           {"start_angle", o.start_scan_angle},
           {"end_angle", o.end_scan_angle},
           {"T_BS", o.T_BS}};
}
// Vehicle_t
inline void to_json(json &j, const Vehicle_t &o) {
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"size", o.size},
           {"cameras", o.cameras},
           {"eventcameras", o.eventcameras},
           {"lidars", o.lidars},
           {"hasCollisionCheck", o.has_collision_check}};
}

// Object_t
inline void to_json(json &j, const Object_t &o) {
  j = json{{"ID", o.ID},
           {"prefabID", o.prefab_ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"size", o.size}};
}

// Setting messages, pub to unity
inline void to_json(json &j, const SettingsMessage_t &o) {
  j = json{
    {"scene_id", o.scene_id}, {"vehicles", o.vehicles}, {"objects", o.objects}};
}

// Publish messages to unity
inline void to_json(json &j, const PubMessage_t &o) {
  j = json{
    {"frame_id", o.frame_id}, {"vehicles", o.vehicles}, {"objects", o.objects}};
}

// Publish messages to unity
inline void from_json(const json &j, Sub_Vehicle_t &o) {
  o.collision = j.at("collision").get<bool>();
  o.lidar_ranges = j.at("lidar_ranges").get<std::vector<Scalar>>();
}

// json to our sub message data type
// note: pub_vechicles is defined in Unity which corresponding
// to our sub_vehicles in ROS.
inline void from_json(const json &j, SubMessage_t &o) {
  o.frame_id = j.at("frame_id").get<uint64_t>();
  o.sub_vehicles = j.at("pub_vehicles").get<std::vector<Sub_Vehicle_t>>();
}

inline void from_json(const json &j, Event_t &o) {
  o.coord_x = j.at("coord_x").get<int>();
  o.coord_y = j.at("coord_y").get<int>();
  o.polarity = j.at("polarity").get<int>();
  o.time = j.at("time").get<float>();
}

inline void from_json(const json &j, EventsMessage_t &o) {
  o.events = j.at("events").get<std::vector<Event_t>>();
}

inline void from_json(const json &j, TimeMessage_t &o) {
  o.current_time = j.at("current_time").get<int64_t>();
  o.next_timestep = j.at("next_timestep").get<int64_t>();
  o.rgb_frame = j.at("rgb_frame").get<bool>();
}

inline void to_json(json &j, const PointCloudMessage_t &o) {
  j = json{{"range", o.range},
           {"origin", o.origin},
           {"resolution", o.resolution},
           {"path", o.path},
           {"file_name", o.file_name}};
}

inline void to_json(json &j, const Event_t &o) {
  j = json{{"coord_x", o.coord_x},
           {"coord_y", o.coord_y},
           {"polarity", o.polarity},
           {"time", o.time}};
}

inline void to_json(json &j, const EventsMessage_t &o) {
  j = json{{"events", o.events}};
}

// Struct for outputting parsed received messages to handler functions
struct RenderMessage_t {
  SubMessage_t sub_msg;
  std::vector<cv::Mat> images;
  // std::vector<Event_t> events;
};

}  // namespace flightlib