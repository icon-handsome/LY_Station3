#include "scan_tracking/flow_control/handlers/inspection_handler.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/detail/state_machine_internal.h"

namespace scan_tracking::flow_control {

const char* InspectionHandler::triggerName() const { return "Trig_Inspection"; }
int InspectionHandler::trigOffset() const { return 24; }

void InspectionHandler::execute(TaskHandlerContext& ctx)
{
    QString algorithm;
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        algorithm = cfgMgr->activePathAlgorithm();
    }

    if (algorithm == QLatin1String("code_read")) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("Trig_Inspection：path algorithm=code_read，走编号识别。");
        ctx.host.startCodeReadCapture();
        return;
    }

    // 测量算法：PLC 假成功放行 + 后台解算；真结果仅 HMI。
    ctx.host.releaseInspectionAndSolveInBackground();
}

}  // namespace scan_tracking::flow_control
