#include "scan_tracking/flow_control/handlers/telescopic_scan_handler.h"

#include "scan_tracking/flow_control/handlers/scan_capture_common.h"

#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking::flow_control {

const char* TelescopicScanHandler::triggerName() const { return "Trig_TelescopicScan"; }
int TelescopicScanHandler::trigOffset() const
{
    return protocol::registers::kTrigTelescopicScan;
}

bool TelescopicScanHandler::isEnabled(const common::StationProfile& profile) const
{
    return profile.enableTelescopicScan && isTriggerEnabledForProfile(profile, triggerName());
}

void TelescopicScanHandler::execute(TaskHandlerContext& ctx)
{
    executeConfiguredScanCapture(ctx, triggerName());
}

}  // namespace scan_tracking::flow_control
