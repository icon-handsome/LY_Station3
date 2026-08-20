#pragma once

#include "scan_tracking/common/station_profile.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/flow_control/task_handler_context.h"

namespace scan_tracking {
namespace flow_control {

class ITaskHandler {
public:
    virtual ~ITaskHandler() = default;
    virtual const char* triggerName() const = 0;
    virtual int trigOffset() const = 0;
    virtual bool isEnabled(const common::StationProfile& profile) const
    {
        return isTriggerEnabledForProfile(profile, triggerName());
    }
    virtual void execute(TaskHandlerContext& ctx) = 0;
};

}  // namespace flow_control
}  // namespace scan_tracking
