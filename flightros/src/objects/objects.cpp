// ros
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/static_object.hpp"

using namespace flightlib;

// Split the string "[pre][sep][post]" into the pair ("[pre]","[post]").
// Example: split("hello/world", "/") will result into ("hello","world").
std::pair<std::string, std::string> split(const std::string& s,
                                          const std::string& sep) {
  auto idx = s.find(sep);
  if (idx == std::string::npos)
    return {s, ""};
  else
    return {s.substr(0, idx), s.substr(idx + 1)};
}

Vector<3> ros2flightmare(const geometry_msgs::Point& p) {
  return Vector<3>(p.x, p.y, p.z);
}

Quaternion ros2flightmare(const geometry_msgs::Quaternion& q) {
  return Quaternion(q.w, q.x, q.y, q.z);
}


// This class allows to add and move objects in Unity3D.
class ObjectDatabase {
 public:
  // Connect to the pose topic.
  ObjectDatabase(ros::NodeHandle& nh,
                 std::shared_ptr<UnityBridge> unity_bridge_ptr) {
    if (unity_bridge_ptr == nullptr) {
      throw std::runtime_error("ObjectDatabase: unity_bridge_ptr is nullptr");
    }
    bridge_ = unity_bridge_ptr;
    sub_ = nh.subscribe("update_object", 1, &ObjectDatabase::poseCB, this);
  }

  std::string getTopic() const { return sub_.getTopic(); }

 private:
  std::shared_ptr<UnityBridge> bridge_;
  std::map<std::string, std::shared_ptr<StaticObject>> database_;
  ros::Subscriber sub_;

  // Update the database.
  void poseCB(const geometry_msgs::PoseStamped& msg) {
    // discard messages with empty frame_id
    if (msg.header.frame_id.empty()) {
      ROS_WARN("Received pose message with empty frame_id");
      return;
    }
    // split the header, which should be in the form "name/type" or at least
    // "name" or "name/".
    auto id = split(msg.header.frame_id, "/");
    // discard messages with missing object name
    if (id.first.empty()) {
      ROS_WARN_STREAM("Failed to deduce an object name from the frame_id "
                      << msg.header.frame_id);
      return;
    }
    // try to locate the object in the database
    std::shared_ptr<StaticObject> object;
    auto obj_it = database_.find(id.first);
    if (obj_it == database_.end()) {
      // create the object if it does not exist yet
      const auto& prefab = id.second.empty() ? id.first : id.second;
      object.reset(new StaticObject(id.first, prefab));
      database_[id.first] = object;
      bridge_->addStaticObject(object);
    } else {
      // since the object exists, just grab it
      object = obj_it->second;
    }
    // update the object
    object->setPosition(ros2flightmare(msg.pose.position));
    object->setQuaternion(ros2flightmare(msg.pose.orientation));
  }
};


int main(int argc, char* argv[]) {
  // initialize ROS
  ros::init(argc, argv, "objects_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate rate(30.0);

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  unity_bridge_ptr->addQuadrotor(std::make_shared<Quadrotor>());
  SceneID scene_id{UnityScene::INDUSTRIAL};
  bool unity_ready = unity_bridge_ptr->connectUnity(scene_id);
  ROS_WARN_COND(!unity_ready, "Failed to connect to Unity3D");

  ObjectDatabase db(pnh, unity_bridge_ptr);

  // Print the usage instructions since the way this node works is
  // not that intuitive (at least the first time!)
  ROS_INFO_STREAM(
    "\n\n------------- USAGE -------------\n\n"
    "You can publish on the topic " +
      db.getTopic() +
      " messages of type "
      "geometry_msgs/PoseStamped in order to add and move static objects "
      "around.\n"
      "To select an object, you must use the frame_id of the message, which "
      "should "
      "take the form 'name[/prefab]'. If the object does not exist yet, it is "
      "created and added to the simulation. If only the name is given, then it "
      "is "
      "also used as type. Otherwise, the explicit type gien after the '/' is "
      "used."
      "\n\nA couple of example messages:\n\n"
      "Add object 'cube' of type 'Transparen_Cube':\n"
      "rostopic pub -1 "
    << db.getTopic()
    << " geometry_msgs/PoseStamped \"{header: "
       "{seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: "
       "'cube/Transparen_Cube'}, "
       "pose: {position: {x: -3, y: 0, z: 1.5}, orientation: {x: 0, y: 0, z: "
       "0, w: 1}"
       "}}\""
       "\n\nChange the position of 'cube':\n"
       "rostopic pub -1 "
    << db.getTopic()
    << " geometry_msgs/PoseStamped \"{header: "
       "{seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'cube'}, pose: "
       "{position: "
       "{x: -1.5, y: 0, z: 1.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}}\""
       "\n\nAdd object 'rpg_gate' of type 'rpg_gate':\n"
       "rostopic pub -1 "
    << db.getTopic()
    << " geometry_msgs/PoseStamped \"{header: "
       "{seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'rpg_gate'}, pose: "
       "{position: "
       "{x: 3, y: 0, z: 1.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}}\"");

  FrameID frame_id = 0;
  while (ros::ok() && unity_ready) {
    ros::spinOnce();
    unity_bridge_ptr->getRender(frame_id);
    unity_bridge_ptr->handleOutput();
    frame_id += 1;
    rate.sleep();
  }

  return 0;
}
