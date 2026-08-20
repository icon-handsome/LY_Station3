#include "scan_tracking/flow_control/handlers/self_check_handler.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

namespace {

// SelfCheck_Fail_Word0 位域（与协议 7.5 对齐并扩展现场回零/CXP）
constexpr quint16 kFailArmMech = 1u << 0;
constexpr quint16 kFailHome = 1u << 1;
constexpr quint16 kFailCxpOrVision = 1u << 2;
constexpr quint16 kFailModbus = 1u << 3;

bool isTurntableAndTelescopicHome(const QVector<quint16>& commandBlock, QString* detail)
{
    const quint16 telescopicStatus =
        commandBlock.value(protocol::registers::kTelescopicRodStatus, 0);
    const quint16 rollerRunHz =
        commandBlock.value(protocol::registers::kRollerRunFreqHz, 0);
    const quint16 robotStatus =
        commandBlock.value(protocol::registers::kRobotStatusWord, 0);
    const bool robotMoving =
        (robotStatus & protocol::registers::robot_status_bits::kRobotMoving) != 0;

    // 伸缩杆：0=待机视为回零/停稳；滚轮运行频率 0 视为转盘静止。
    const bool telescopicHome = (telescopicStatus == 0);
    const bool turntableHome = (rollerRunHz == 0);
    const bool ok = telescopicHome && turntableHome && !robotMoving;

    if (detail != nullptr) {
        *detail = QStringLiteral("伸缩杆状态=%1 滚轮运行Hz=%2 机械臂运动=%3")
                      .arg(telescopicStatus)
                      .arg(rollerRunHz)
                      .arg(robotMoving ? QStringLiteral("是") : QStringLiteral("否"));
    }
    return ok;
}

}  // namespace

const char* SelfCheckHandler::triggerName() const { return "Trig_SelfCheck"; }
int SelfCheckHandler::trigOffset() const { return 26; }

void SelfCheckHandler::execute(TaskHandlerContext& ctx)
{
    auto* mechArm = ctx.host.mechEyeArmService();
    auto* vision = ctx.host.visionPipelineService();

    const bool modbusReady = ctx.host.isModbusConnected();
    const bool mechArmReady =
        mechArm != nullptr && mechArm->state() != mech_eye::CameraRuntimeState::Error;
    const bool visionReady = vision != nullptr && vision->isStarted();

    const auto* configMgr = common::ConfigManager::instance();
    const bool hikCxpEnabled =
        configMgr != nullptr && configMgr->visionConfig().hikCxpEnabled;
    const bool hikCxpBypassOk =
        configMgr != nullptr && configMgr->visionConfig().hikCxpBypassOk;
    // 跳过 CXP 采图时不要求采集卡就绪；段扫/自检仍正常进入。
    const bool cxpGateOk = hikCxpBypassOk || hikCxpEnabled;

    QString homeDetail;
    const bool homeOk = isTurntableAndTelescopicHome(ctx.commandBlock, &homeDetail);

    quint16 failWord0 = 0;
    if (!mechArmReady) {
        failWord0 |= kFailArmMech;
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：机械臂梅卡不可用。");
    }
    if (!homeOk) {
        failWord0 |= kFailHome;
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("自检：转盘/伸缩杆未回零或机械臂仍在运动：") << homeDetail;
    }
    if (!visionReady || !cxpGateOk) {
        failWord0 |= kFailCxpOrVision;
        if (!visionReady) {
            qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：视觉流水线不可用。");
        }
        if (!cxpGateOk) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("自检：hikCxpEnabled=false 且 hikCxpBypassOk=false，无法进行自检采集。");
        }
    }
    if (!modbusReady) {
        failWord0 |= kFailModbus;
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Modbus 不可用。");
    }

    if (failWord0 != 0) {
        if (modbusReady) {
            ctx.host.writeSelfCheckFailWords({failWord0});
        }
        // 协议：Res_SelfCheck=2 未通过
        ctx.host.completeActiveTask(2, protocol::AckState::Failed, false);
        ctx.host.notifySelfCheckFinished(2, failWord0);
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("自检前置通过：") << homeDetail
        << QStringLiteral(" 段号=") << ctx.activeTask.scanSegmentIndex
        << QStringLiteral("，发起机械臂 3D+CXP 采集（不用智能相机）。");

    ctx.host.writeSelfCheckFailWords({0});
    ctx.host.startSelfCheckCapture();
}

}  // namespace scan_tracking::flow_control
