#pragma once

#include "scan_tracking/common/station_profile.h"

namespace scan_tracking {
namespace flow_control {

bool isTriggerEnabledForProfile(const common::StationProfile& profile, const char* triggerName);
bool isTriggerEnabledForProfile(const common::StationProfile& profile, int trigOffset);

}  // namespace flow_control
}  // namespace scan_tracking
