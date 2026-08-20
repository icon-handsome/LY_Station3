#include "scan_tracking/flow_control/handlers/scan_segment_handler.h"

#include "scan_tracking/flow_control/handlers/scan_capture_common.h"

namespace scan_tracking::flow_control {

const char* ScanSegmentHandler::triggerName() const { return "Trig_ScanSegment"; }
int ScanSegmentHandler::trigOffset() const { return 23; }

void ScanSegmentHandler::execute(TaskHandlerContext& ctx)
{
    executeConfiguredScanCapture(ctx, triggerName());
}

}  // namespace scan_tracking::flow_control
