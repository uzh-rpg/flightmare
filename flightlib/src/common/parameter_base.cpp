#include "flightlib/common/parameter_base.hpp"

namespace flightlib {

ParameterBase::ParameterBase() {}

ParameterBase::ParameterBase(const YAML::Node& cfg_node)
  : cfg_node_(cfg_node) {}

ParameterBase::ParameterBase(const std::string& cfg_path)
  : cfg_node_(YAML::Node(cfg_path)) {}

ParameterBase::~ParameterBase() {}

}  // namespace flightlib
