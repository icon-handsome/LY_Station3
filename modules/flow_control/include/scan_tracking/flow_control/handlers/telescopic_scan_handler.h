#pragma once

#include "scan_tracking/flow_control/handlers/itask_handler.h"

namespace scan_tracking {
namespace flow_control {

class TelescopicScanHandler final : public ITaskHandler {
public:
    const char* triggerName() const override;
    int trigOffset() const override;
    bool isEnabled(const common::StationProfile& profile) const override;
    void execute(TaskHandlerContext& ctx) override;
};

}  // namespace flow_control
}  // namespace scan_tracking
