#include "scan_tracking/flow_control/handlers/code_read_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

namespace scan_tracking::flow_control {

const char* CodeReadHandler::triggerName() const { return "Trig_CodeRead"; }
int CodeReadHandler::trigOffset() const { return 27; }

void CodeReadHandler::execute(TaskHandlerContext& ctx)
{
    qInfo(LOG_FLOW).noquote() << QStringLiteral("收到 Trig_CodeRead，发起海康 C 编号识别。");
    ctx.host.startCodeReadCapture();
}

}  // namespace scan_tracking::flow_control
