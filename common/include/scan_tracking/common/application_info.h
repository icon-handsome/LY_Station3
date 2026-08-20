#pragma once

#include <string>

namespace scan_tracking {
namespace common {

class ApplicationInfo {
public:
    static std::string name();
    static std::string version();
};

}  // namespace common
}  // namespace scan_tracking
