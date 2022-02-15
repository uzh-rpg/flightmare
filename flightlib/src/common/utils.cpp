#include "flightlib/common/utils.hpp"

namespace flightlib {

bool file_exists(const std::filesystem::path& p) {
  return (std::filesystem::exists(p));
}

}  // namespace flightlib