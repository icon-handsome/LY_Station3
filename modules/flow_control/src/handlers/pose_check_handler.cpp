#include "scan_tracking/flow_control/handlers/pose_check_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/lb_pose_check.h"

namespace scan_tracking::flow_control {

const char* PoseCheckHandler::triggerName() const { return "Trig_PoseCheck"; }
int PoseCheckHandler::trigOffset() const { return 22; }

void PoseCheckHandler::execute(TaskHandlerContext& ctx)
{
    const QVector<double> identityRt = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    const auto* configMgr = common::ConfigManager::instance();
    if (configMgr == nullptr) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Trig_PoseCheck：ConfigManager 不可用。");
        ctx.host.writeFloatPlaceholder(protocol::registers::kPoseDeviationMm, 0.0f);
        ctx.host.completeActiveTask(7, protocol::AckState::Failed, false);
        ctx.host.notifyPoseCheckFinished(
            false, 7, 0.0, identityRt, QStringLiteral("配置不可用"));
        return;
    }

    const tracking::PoseCheckResult poseResult =
        tracking::runLegacyLbPoseCheck(configMgr->lbPoseConfig());

    ctx.host.writeFloatPlaceholder(
        protocol::registers::kPoseDeviationMm,
        static_cast<float>(poseResult.poseDeviationMm));

    QVector<double> rt;
    rt.reserve(static_cast<int>(poseResult.rt.size()));
    for (double value : poseResult.rt) {
        rt.append(value);
    }

    if (!poseResult.invoked) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Trig_PoseCheck：未调用 LB：") << poseResult.message;
        ctx.host.completeActiveTask(7, protocol::AckState::Failed, false);
        ctx.host.notifyPoseCheckFinished(
            false, 7, poseResult.poseDeviationMm, identityRt, poseResult.message);
        return;
    }

    if (!poseResult.success) {
        const quint16 resultCode = poseResult.resultCode == 0 ? 7 : poseResult.resultCode;
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Trig_PoseCheck：失败") << poseResult.message
            << QStringLiteral(" resultCode=") << resultCode;
        ctx.host.completeActiveTask(resultCode, protocol::AckState::Failed, false);
        ctx.host.notifyPoseCheckFinished(
            false, resultCode, poseResult.poseDeviationMm, rt, poseResult.message);
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_PoseCheck：成功")
        << QStringLiteral(" inputPoints=") << poseResult.inputPointCount
        << QStringLiteral(" deviationMm=") << poseResult.poseDeviationMm;
    ctx.host.completeActiveTask(1, protocol::AckState::Completed, true);
    ctx.host.notifyPoseCheckFinished(
        true, 1, poseResult.poseDeviationMm, rt, poseResult.message);
}

}  // namespace scan_tracking::flow_control
