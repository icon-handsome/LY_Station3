#include "scan_tracking/flow_control/station_trigger_policy.h"

#include <cstring>

#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking {
namespace flow_control {

bool isTriggerEnabledForProfile(const common::StationProfile& profile, const char* triggerName)
{
    if (triggerName == nullptr) {
        return false;
    }
    if (std::strcmp(triggerName, "Trig_LoadGrasp") == 0) {
        return profile.enableLoadGrasp;
    }
    if (std::strcmp(triggerName, "Trig_UnloadCalc") == 0) {
        return profile.enableUnloadCalc;
    }
    if (std::strcmp(triggerName, "Trig_PoseCheck") == 0) {
        return profile.enablePoseCheck;
    }
    if (std::strcmp(triggerName, "Trig_TelescopicScan") == 0) {
        return profile.enableTelescopicScan;
    }

    // Stage 2 only gates the three first-station-only triggers above.
    // enableTelescopicScan / enableHoistAssist / enableCollisionMonitor are stage3+ switches.
    return true;
}

bool isTriggerEnabledForProfile(const common::StationProfile& profile, int trigOffset)
{
    const auto* trigger = protocol::triggerByOffset(trigOffset);
    return trigger != nullptr && isTriggerEnabledForProfile(profile, trigger->name);
}

}  // namespace flow_control
}  // namespace scan_tracking
