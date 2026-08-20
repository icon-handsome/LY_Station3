#include "scan_tracking/flow_control/handlers/unload_calc_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking::flow_control {

const char* UnloadCalcHandler::triggerName() const { return "Trig_UnloadCalc"; }
int UnloadCalcHandler::trigOffset() const { return 25; }

void UnloadCalcHandler::execute(TaskHandlerContext& ctx)
{
    const auto poseSource = ctx.host.resolveUnloadCalcPoseSource();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("UnloadCalc 位姿来源：")
        << poseSource.sourceName
        << QStringLiteral(" 可用=") << poseSource.available
        << QStringLiteral(" 成功=") << poseSource.success
        << QStringLiteral(" 消息=") << poseSource.message;

    ctx.host.writeUnloadCalcResult(poseSource);
    const quint16 resultCode = poseSource.success ? 1 : 7;
    ctx.host.completeActiveTask(
        resultCode,
        poseSource.success ? protocol::AckState::Completed : protocol::AckState::Failed,
        poseSource.success);
    ctx.host.notifyUnloadCalcFinished(resultCode, poseSource);
}

}  // namespace scan_tracking::flow_control
