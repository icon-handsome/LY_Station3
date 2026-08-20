#pragma once

#include "scan_tracking/flow_control/task_handler_context.h"

namespace scan_tracking {
namespace flow_control {

/// 按 scan_paths 段号发起采集；机械臂/伸缩杆相机组由 PLC 触发器（ScanSegment / TelescopicScan）决定。
void executeConfiguredScanCapture(TaskHandlerContext& ctx, const char* triggerLabel);

}  // namespace flow_control
}  // namespace scan_tracking
