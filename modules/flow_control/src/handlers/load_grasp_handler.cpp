#include "scan_tracking/flow_control/handlers/load_grasp_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking::flow_control {

const char* LoadGraspHandler::triggerName() const { return "Trig_LoadGrasp"; }
int LoadGraspHandler::trigOffset() const { return 20; }

void LoadGraspHandler::execute(TaskHandlerContext& ctx)
{
    const auto poseSource = ctx.host.resolveLoadGraspPoseSource();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("LoadGrasp 位姿来源：")
        << poseSource.sourceName
        << QStringLiteral(" 可用=") << poseSource.available
        << QStringLiteral(" 成功=") << poseSource.success
        << QStringLiteral(" 消息=") << poseSource.message;

    ctx.host.writeLoadGraspResult(poseSource);
    const quint16 resultCode = poseSource.success ? 1 : 7;
    ctx.host.completeActiveTask(
        resultCode,
        poseSource.success ? protocol::AckState::Completed : protocol::AckState::Failed,
        poseSource.success);
    ctx.host.notifyLoadGraspFinished(resultCode, poseSource);
}

}  // namespace scan_tracking::flow_control
