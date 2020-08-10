#pragma once


#include <yaml-cpp/yaml.h>

namespace flightlib {

struct ParameterBase {
  ParameterBase();
  ParameterBase(const std::string& cfg_path);
  ParameterBase(const YAML::Node& cfg_node);

  virtual ~ParameterBase();

  virtual bool valid() = 0;
  virtual bool loadParam() = 0;

  YAML::Node cfg_node_;
};

}  // namespace flightlib
