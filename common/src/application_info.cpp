#include "scan_tracking/common/application_info.h"

#include <string>

namespace scan_tracking {
namespace common {

std::string ApplicationInfo::name() {
    return "Scan Tracking";
}

std::string ApplicationInfo::version() {
    return SCAN_TRACKING_VERSION;
}

}  // namespace common
}  // namespace scan_tracking
