#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

namespace {

/// 忙碌期间锁存的 Trig 上升沿（bit=trigOffset）。放文件静态，避免改 StateMachine 布局。
quint64 g_latchedTrigRisingMask = 0;

/// 扫描相机齐套边沿日志（文件静态，避免改 StateMachine 成员布局导致增量/ABI 闪退）。
bool g_lastScanCamerasReadyForPlc = false;
bool g_hasLoggedScanCamerasReadyForPlc = false;

/// 因相机未齐套主动关闭 Modbus 监听时置位，避免走「Modbus 断开故障」。
bool g_expectModbusDisconnectForCameraGate = false;

/// Mech + 海康智能 C 在线后即可向 PLC 刷新心跳 / 开 Modbus。
/// CXP 不参与在线门控（仍可后台连接，状态日志照常打印）。
bool scanCamerasReadyForPlcOnline(const StateMachine& sm, QString* missingDetail)
{
    QStringList missing;

    const auto requireMech = [&](mech_eye::MechEyeService* service, const QString& label) {
        if (service == nullptr) {
            return;
        }
        if (!service->isCameraConnected()) {
            missing << label;
        }
    };
    requireMech(sm.mechEyeArmService(), QStringLiteral("Mech-机械臂"));
    requireMech(sm.mechEyeTelescopicService(), QStringLiteral("Mech-伸缩杆"));

    const auto* configMgr = common::ConfigManager::instance();
    vision::HikCameraCController* hikController = sm.hikCameraCController();
    if (hikController == nullptr) {
        missing << QStringLiteral("海康智能C控制器");
    } else if (configMgr != nullptr) {
        const auto& vision = configMgr->visionConfig();
        const auto requireHikSmart = [&](const QString& ip, const QString& label) {
            const QString trimmed = ip.trimmed();
            if (trimmed.isEmpty()) {
                return;
            }
            if (!hikController->isCameraConnected(trimmed)) {
                missing << QStringLiteral("%1(%2)").arg(label, trimmed);
            }
        };
        requireHikSmart(vision.armGroup.hikCameraC.ipAddress, QStringLiteral("海康智能C-机械臂"));
        requireHikSmart(vision.telescopicGroup.hikCameraC.ipAddress, QStringLiteral("海康智能C-伸缩杆"));
    } else if (!hikController->isCameraConnectedToTcp()) {
        missing << QStringLiteral("海康智能C");
    }

    if (missingDetail != nullptr) {
        *missingDetail = missing.join(QStringLiteral(", "));
    }
    return missing.isEmpty();
}

}  // namespace

void StateMachine::onModbusConnected()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    QString missingCameras;
    if (!scanCamerasReadyForPlcOnline(*this, &missingCameras)) {
        // 海康心跳抖动等导致短暂未齐套时：保持 Modbus，不主动 disconnect。
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Modbus 已连接但扫描相机暂未齐套，保持监听：")
            << missingCameras;
    }

    qInfo(LOG_FLOW) << QStringLiteral("Modbus 已连接，流程控制就绪。");

    if (m_activeTask.definition != nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Modbus 重连后清除残留活动任务：")
            << protocol::triggerName(*m_activeTask.definition);
        clearActiveTask();
    }

    // 重连后丢弃旧命令块；首帧若 Trig 仍为 1 且空闲则补接受（见 onCommandBlockUpdated）。
    m_lastCommandBlock.clear();
    m_blockTrigUntilIdleOffset = -1;
    g_latchedTrigRisingMask = 0;
    m_advancePathAfterTriggerRelease = false;
    m_codeReadPending = false;
    m_codeReadSoftPending = false;
    m_codeReadCameraIp.clear();

    m_isPollingPlc = false;
    resetModbusFailureCounter();
    m_consecutiveModbusFailures = 0;
    m_ipcState = protocol::IpcState::Ready;
    m_currentStage = protocol::Stage::Idle;
    m_alarmLevel = 0;
    m_alarmCode = 0;
    m_warnCode = 0;
    m_progress = 0;
    m_dataValid = false;
    setState(AppState::Ready);
    publishIpcStatus();
    publishHeartbeat();
    m_pollTimer->start();
    m_heartbeatTimer->start();

    qInfo(LOG_FLOW) << QStringLiteral("Modbus 重连恢复完成，系统已回到就绪状态。");
}

void StateMachine::onModbusDisconnected()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    m_pollTimer->stop();
    m_timeoutTimer->stop();
    m_isPollingPlc = false;

    // 相机未齐套时主动关监听：不算故障，心跳定时器继续跑以便齐套后重新 listen。
    if (g_expectModbusDisconnectForCameraGate) {
        g_expectModbusDisconnectForCameraGate = false;
        m_ipcState = protocol::IpcState::Initializing;
        setState(AppState::Init);
        if (m_heartbeatTimer != nullptr && !m_heartbeatTimer->isActive()) {
            m_heartbeatTimer->start();
        }
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("已关闭 Modbus 监听（扫描相机未齐套，PLC 无法连接 IPC）");
        return;
    }

    qWarning(LOG_FLOW) << QStringLiteral("Modbus 已断开，流程控制暂停。");
    m_heartbeatTimer->stop();
    enterFaultState(900, QStringLiteral("Modbus 已断开连接"), true, false);
}

void StateMachine::onModbusError(const QString& errorString)
{
    qWarning(LOG_FLOW).noquote() << "Modbus 错误传播到流程控制：" << errorString;
    recordModbusFailure(901, errorString);
}

void StateMachine::pollPlcState()
{
}

void StateMachine::handleRegistersRead(int startAddress, const QVector<quint16>& values)
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (startAddress != protocol::registers::kCommandBlockStart ||
        values.size() < protocol::registers::kCommandBlockSize) {
        return;
    }

    const QVector<quint16> previousCommandBlock = m_lastCommandBlock;
    m_lastCommandBlock = values;
    m_robotTcpPose = protocol::registers::readRobotTcpPoseFromCommandBlock(values);
    resetModbusFailureCounter();

    bool commandBlockChanged = previousCommandBlock.isEmpty();
    if (!commandBlockChanged) {
        const int compareCount = qMin(previousCommandBlock.size(), values.size());
        for (int index = 1; index < compareCount; ++index) {
            if (previousCommandBlock.value(index) != values.value(index)) {
                commandBlockChanged = true;
                break;
            }
        }
    }

    if (m_activePollRequestSequence == 1 || (m_activePollRequestSequence % kPollLogEveryN) == 0) {
        qDebug(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询完成")
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence
            << QStringLiteral(" 耗时ms=") << (m_pollRequestTimer.isValid() ? m_pollRequestTimer.elapsed() : -1);
    }
    m_activePollRequestSequence = 0;

    if (commandBlockChanged) {
        namespace regs = protocol::registers;
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("命令块快照：")
            << "Flow_Enable=" << values.value(regs::kFlowEnable)
            << "ArmScanSegmentIndex(AO47)="
            << regs::plcAnalogToUInt16(values.value(regs::kArmScanSegmentIndex), 0)
            << "TelescopicScanSegmentIndex(AO48)="
            << regs::plcAnalogToUInt16(values.value(regs::kTelescopicScanSegmentIndex), 0)
            << "Trig_ScanSegment=" << values.value(regs::modbusIndexFromPlcAddress(40023))
            << "Trig_TelescopicScan=" << values.value(regs::kTrigTelescopicScan)
            << "Trig_Inspection=" << values.value(regs::modbusIndexFromPlcAddress(40024))
            << "ScanPathId="
            << regs::plcAnalogToUInt16(values.value(regs::kScanPathId), 0)
            << "TaskIdHigh=" << values.value(regs::kTaskIdHigh)
            << "TaskIdLow=" << values.value(regs::kTaskIdLow);
    }

    if (!previousCommandBlock.isEmpty()) {
        static const char* const kRegisterNames[] = {
            "Reserved_0", "PLC_Heartbeat", "PLC_SystemState", "Station_WorkMode", "Flow_Enable",
            "Safety_Status_Word", "Cmd_StartAuto", "Cmd_Pause", "Cmd_Stop", "Cmd_Reset",
            "Cmd_ClearAlarms", "TaskId_H", "TaskId_L", "ProductType", "RecipeId",
            "ArmScanSegmentIndex_AO47", "TelescopicScanSegmentIndex_AO48", "RequestTimeout_s", "Robot_Status_Word",
            "Reserved_CmdExt_19", "Trig_LoadGrasp", "Trig_StationMaterialCheck", "Trig_PoseCheck",
            "Trig_ScanSegment", "Trig_Inspection", "Trig_UnloadCalc", "Trig_SelfCheck",
            "Trig_CodeRead", "Trig_ResultReset",
            "RobotTcpX_w0", "RobotTcpX_w1", "RobotTcpY_w0", "RobotTcpY_w1", "RobotTcpZ_w0",
            "RobotTcpZ_w1", "RobotTcpRx_w0", "RobotTcpRx_w1", "RobotTcpRy_w0", "RobotTcpRy_w1",
            "RobotTcpRz_w0", "RobotTcpRz_w1",
            "TelescopicRodStatus", "RollerSetFreqHz", "RollerRunFreqHz", "ElectromagnetStatus",
            "EstopButtonStatus", "Trig_TelescopicScan", "ScanPathId", "Reserved_CmdExt_48",
            "Reserved_CmdExt_49", "Reserved_CmdExt_50",
        };
        constexpr int kNameCount = sizeof(kRegisterNames) / sizeof(kRegisterNames[0]);
        const int compareCount = qMin(previousCommandBlock.size(),
                                      qMin(values.size(), protocol::registers::kCommandBlockSize));

        QStringList changedFields;
        for (int index = 0; index < compareCount; ++index) {
            const quint16 oldValue = previousCommandBlock.value(index);
            const quint16 newValue = values.value(index);
            if (oldValue == newValue) {
                continue;
            }
            const char* name = (index < kNameCount) ? kRegisterNames[index] : "?";
            changedFields << QStringLiteral("  [%1] %2: %3")
                                 .arg(protocol::registers::holdingRegisterAddress(index))
                                 .arg(QString::fromLatin1(name))
                                 .arg(formatPlcRegisterChangeForLog(index, oldValue, newValue));
        }
        if (!changedFields.isEmpty()) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("=== PLC 寄存器变化 ===") << "\n"
                << changedFields.join(QStringLiteral("\n"));
        }
    }

    if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
        finalizeCompletedTaskIfTriggerReleased(values);

        // 完成时 Trig 已是 0，但中间无 PLC 写事件；本次又看到同触发 0→1，
        // 视为「已释放 + 新触发」，先收尾旧任务再往下接受。
        if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
            const int trigOffset = m_activeTask.definition->trigOffset;
            if (trigOffset >= 0 &&
                trigOffset < previousCommandBlock.size() &&
                trigOffset < values.size() &&
                previousCommandBlock.value(trigOffset) == 0 &&
                values.value(trigOffset) == 1) {
                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("任务已完成且 Trig 重新置位，强制释放旧任务：")
                    << protocol::triggerName(*m_activeTask.definition);
                finalizeCompletedTaskIfTriggerReleased(values, true);
            }
        }
    }

    // 任意 Trig 的 0→1：忙碌时也锁存，避免臂扫进行中错过伸缩杆上升沿后永久丢触发。
    if (!previousCommandBlock.isEmpty()) {
        for (const auto& trigger : protocol::triggerDefinitions()) {
            if (trigger.trigOffset < 0 ||
                trigger.trigOffset >= 64 ||
                trigger.trigOffset >= values.size() ||
                trigger.trigOffset >= previousCommandBlock.size()) {
                continue;
            }
            const quint16 prev = previousCommandBlock.value(trigger.trigOffset);
            const quint16 curr = values.value(trigger.trigOffset);
            const quint64 bit = 1ull << static_cast<unsigned>(trigger.trigOffset);
            if (curr == 0) {
                g_latchedTrigRisingMask &= ~bit;
            } else if (prev == 0 && curr == 1) {
                g_latchedTrigRisingMask |= bit;
            }
        }
    }

    // 强制收尾后的闭锁：该 Trig 必须先回到 0 才允许再次接受。
    // offset 必须是真实 Trig 下标（>0）；0 为保留字，避免误判「解除闭锁」。
    if (m_blockTrigUntilIdleOffset > 0 &&
        m_blockTrigUntilIdleOffset < values.size() &&
        values.value(m_blockTrigUntilIdleOffset) == 0) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("Trig 已回 0，解除重复触发闭锁 offset=")
            << m_blockTrigUntilIdleOffset;
        m_blockTrigUntilIdleOffset = -1;
    }

    if (m_activeTask.definition != nullptr) {
        return;
    }

    // 空闲时：仅在 40047 相对上次轮询变化时切路，避免覆盖 IPC 自动切路结果。
    applyPlcScanPathId(values, previousCommandBlock, true);

    // 首帧（previous 为空）：重连/启动后若 Trig 仍保持 1 且空闲，补接受（勿一律当残留丢掉）。
    // 用全 0 伪上一拍制造上升沿，复用 selectPendingTrigger / 锁存逻辑。
    if (previousCommandBlock.isEmpty()) {
        QVector<quint16> syntheticPrevious(values.size(), 0);
        if (const protocol::TriggerDefinition* heldTrigger =
                selectPendingTrigger(values, syntheticPrevious)) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("重连/首帧补接受保持为 1 的触发 ")
                << protocol::triggerName(*heldTrigger)
                << QStringLiteral("（空闲且 Trig=1）");
            if (heldTrigger->trigOffset >= 0 && heldTrigger->trigOffset < 64) {
                g_latchedTrigRisingMask &=
                    ~(1ull << static_cast<unsigned>(heldTrigger->trigOffset));
            }
            processTrigger(*heldTrigger, values);
        }
        return;
    }

    if (const protocol::TriggerDefinition* pendingTrigger =
            selectPendingTrigger(values, previousCommandBlock)) {
        if (pendingTrigger->trigOffset >= 0 && pendingTrigger->trigOffset < 64) {
            g_latchedTrigRisingMask &= ~(1ull << static_cast<unsigned>(pendingTrigger->trigOffset));
        }
        processTrigger(*pendingTrigger, values);
    }
}

void StateMachine::onRegisterReadFailed(int startAddress, const QString& errorString)
{
    if (startAddress != protocol::registers::kCommandBlockStart) {
        return;
    }

    if (m_isPollingPlc) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询失败：") << errorString
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence;
    }
    m_isPollingPlc = false;
    m_activePollRequestSequence = 0;
}

void StateMachine::onRegisterWriteFailed(int startAddress, const QString& errorString)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("寄存器写入失败，地址=") << startAddress << errorString;
}

void StateMachine::processTrigger(const protocol::TriggerDefinition& trigger, const QVector<quint16>& commandBlock)
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    // Trig 前以 PLC ScanPathId 为准（同一次轮询写 path 再置 Trig 也能生效）。
    applyPlcScanPathId(commandBlock);

    if (const auto* configMgr = common::ConfigManager::instance()) {
        const auto& profile = configMgr->stationProfile();
        if (!isTriggerEnabledForProfile(profile, trigger.trigOffset)) {
            rejectDisabledTrigger(trigger);
            return;
        }
    }

    // 双保险：即使心跳竞态下 PLC 仍下发 Trig，相机未齐套也不进入任务（机械臂不会去点位）。
    // ResultReset 仅清结果，允许在相机未齐时执行。
    QString missingCameras;
    if (trigger.stage != protocol::Stage::ResultReset &&
        !scanCamerasReadyForPlcOnline(*this, &missingCameras)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("扫描相机未齐套，拒绝触发：")
            << protocol::triggerName(trigger)
            << QStringLiteral(" 缺失=") << missingCameras;
        if (!isScanCaptureStage(trigger.stage)) {
            sendAck(trigger, protocol::AckState::Running);
        }
        sendRes(trigger, 9);
        sendAck(trigger, protocol::AckState::Failed);
        return;
    }

    if (trigger.stage != protocol::Stage::UnloadCalc &&
        trigger.stage != protocol::Stage::ResultReset &&
        commandBlock.value(protocol::registers::kFlowEnable) == 0) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Flow_Enable=0 时拒绝触发：")
                                     << protocol::triggerName(trigger);
        sendRes(trigger, 9);
        sendAck(trigger, protocol::AckState::Failed);
        return;
    }

    m_activeTask.definition = &trigger;
    m_activeTask.taskId = readTaskId(commandBlock);
    {
        const quint16 timeoutRaw = commandBlock.value(protocol::registers::kRequestTimeoutSeconds);
        const quint16 timeoutDecoded = protocol::registers::plcAnalogToUInt16(timeoutRaw, 0);
        m_activeTask.timeoutSeconds = timeoutDecoded > 0
            ? timeoutDecoded
            : static_cast<quint16>(trigger.defaultTimeoutSeconds);
    }
    m_activeTask.scanSegmentIndex = resolveScanSegmentIndex(commandBlock, trigger.stage);
    m_activeTask.inspectionPathId = 0;

    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        m_activeTask.inspectionPathId = cfgMgr->activePathId();
        // 段号分母用当前触发设备的本地配额（AO47/AO48 各自 1..N）。
        // 路径总点数 = 臂 + 伸缩杆（如 path5 环缝 18+18=36），另在日志中打印，避免与本地段号混淆。
        int configuredTotal = 0;
        if (trigger.stage == protocol::Stage::TelescopicScan) {
            configuredTotal = cfgMgr->enabledTelescopicPointCount();
        } else if (trigger.stage == protocol::Stage::ScanSegment) {
            configuredTotal = cfgMgr->enabledArmPointCount();
        } else {
            configuredTotal = cfgMgr->enabledScanPointCount();
        }
        if (configuredTotal <= 0) {
            configuredTotal = cfgMgr->trackingConfig().scanSegmentTotal;
        }
        m_activeTask.scanSegmentTotal = configuredTotal > 0 ? configuredTotal : 1;
    } else {
        m_activeTask.scanSegmentTotal = 1;
    }
    m_activeTask.completionAnnounced = false;
    m_activeTask.captureRequestId = 0;
    m_activeTask.workpieceGeneration = workpieceGeneration();

    {
        const auto* cfgMgr = common::ConfigManager::instance();
        const int armQuota = cfgMgr != nullptr ? cfgMgr->enabledArmPointCount() : 0;
        const int telescopicQuota =
            cfgMgr != nullptr ? cfgMgr->enabledTelescopicPointCount() : 0;
        const int pathTotal = armQuota + telescopicQuota;
        const QString deviceTag =
            trigger.stage == protocol::Stage::TelescopicScan
                ? QStringLiteral("telescopic")
                : (trigger.stage == protocol::Stage::ScanSegment
                       ? QStringLiteral("arm")
                       : QStringLiteral("-"));
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("已接受触发") << protocol::triggerName(trigger)
            << QStringLiteral(" pathId=") << m_activeTask.inspectionPathId
            << QStringLiteral(" 超时s=") << m_activeTask.timeoutSeconds
            << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex
            << QStringLiteral("/") << m_activeTask.scanSegmentTotal
            << QStringLiteral("(") << deviceTag << QStringLiteral(")")
            << QStringLiteral(" 路径配额 arm=") << armQuota
            << QStringLiteral(" telescopic=") << telescopicQuota
            << QStringLiteral(" total=") << pathTotal
            << QStringLiteral(" workpieceGen=") << m_activeTask.workpieceGeneration;
    }

    setAlarm(0, 0, QString());
    setState(AppState::Scanning);
    m_ipcState = protocol::IpcState::Busy;
    m_currentStage = trigger.stage;
    m_progress = 5;
    m_dataValid = false;
    publishIpcStatus();

    // 段扫（臂/伸缩杆）：与现场 PLC 简化握手对齐——不再发 Ack=1(Running)，
    // 仅在本段全部参与相机拍完后写 Ack=2/3，避免 PLC 见 Running 即离位。
    if (!isScanCaptureStage(trigger.stage)) {
        sendAck(trigger, protocol::AckState::Running);
    } else {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("段扫简化握手：跳过 Ack=1，待全部相机完成后写 Ack=2/3")
            << protocol::triggerName(trigger);
    }

    if (m_activeTask.taskId != 0) {
        const bool taskIdWritten = m_modbus->writeRegisters(protocol::registers::kTaskIdEchoHigh, {
            static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu),
            static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),
        });
        if (!taskIdWritten) {
            qWarning(LOG_FLOW).noquote() << QStringLiteral("写入任务 ID 回声寄存器失败");
        }
    }

    m_timeoutTimer->start(static_cast<int>(m_activeTask.timeoutSeconds) * 1000);
    executeActiveTask();
}

void StateMachine::rejectDisabledTrigger(const protocol::TriggerDefinition& trigger)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[Station] 触发器")
        << protocol::triggerName(trigger)
        << QStringLiteral("在当前 profile 未启用，已拒绝，Res=8");
    // 段扫同样跳过 Ack=1；其它业务仍先 Running 再 Failed，保持原边沿。
    if (!isScanCaptureStage(trigger.stage)) {
        sendAck(trigger, protocol::AckState::Running);
    }
    sendRes(trigger, 8);
    sendAck(trigger, protocol::AckState::Failed);
}

void StateMachine::sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState)
{
    if (!m_modbus) {
        return;
    }

    const bool ackWritten = m_modbus->writeRegister(definition.ackOffset, static_cast<quint16>(ackState));
    if (!ackWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Ack 状态失败");
    }
}

void StateMachine::sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode)
{
    if (!m_modbus) {
        return;
    }

    const bool resWritten = m_modbus->writeRegister(definition.resOffset, resultCode);
    if (!resWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Res 结果码失败");
    }
}

void StateMachine::resetPlcOutputRegisters()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    if (m_modbus->resetIpcResultBlock()) {
        qInfo(LOG_FLOW).noquote() << QStringLiteral("程序退出：IPC 结果区已清零");
    } else {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("程序退出：IPC 结果区清零失败");
    }
}

void StateMachine::publishIpcStatus()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    // 已建链则持续写心跳：海康心跳短暂超时不再冻心跳，避免 PLC 误判 IPC 离线。
    // 新扫触发仍由 processTrigger 在相机未齐套时拒绝。

    QVector<quint16> status = {
        m_heartbeatCounter,
        static_cast<quint16>(m_ipcState),
        static_cast<quint16>(m_currentStage),
        m_alarmLevel,
        m_alarmCode,
        m_warnCode,
        static_cast<quint16>(m_state == AppState::Ready ? 1 : 0),
        static_cast<quint16>(m_dataValid ? 1 : 0),
        m_progress,
        kDeviceOnlineWord0,
        0, 0, 0,
        static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu),
        static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),
    };

    if (!m_modbus->writeRegisters(protocol::registers::kIpcHeartbeat, status)) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 IPC 心跳状态失败");
    }
}

void StateMachine::publishHeartbeat()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (m_modbus == nullptr) {
        return;
    }

    QString missingDetail;
    const bool camerasReady = scanCamerasReadyForPlcOnline(*this, &missingDetail);
    if (camerasReady != g_lastScanCamerasReadyForPlc || !g_hasLoggedScanCamerasReadyForPlc) {
        g_lastScanCamerasReadyForPlc = camerasReady;
        g_hasLoggedScanCamerasReadyForPlc = true;
        if (camerasReady) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("扫描相机已齐套（Mech+海康智能，忽略CXP），启动/恢复 Modbus 与 IPC 心跳");
        } else {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("扫描相机暂未齐套，保持 Modbus 监听与心跳；新扫触发仍会拒绝：")
                << missingDetail;
        }
    }

    // 仅「尚未建链」时用齐套作为首次 listen 门控；已建链后相机抖动不再 disconnect。
    if (!m_modbus->isConnected()) {
        if (!camerasReady) {
            return;
        }
        if (!m_modbus->connectDevice()) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("扫描相机已齐套，但 Modbus 监听启动失败");
        }
        // connectDevice 可能同步触发 onModbusConnected→publishHeartbeat；此处直接返回避免重复计数。
        return;
    }

    ++m_heartbeatCounter;
    publishIpcStatus();
}

void StateMachine::onProcessTimeout()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (m_activeTask.definition == nullptr) {
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("任务超时：") << protocol::triggerName(*m_activeTask.definition);
    setAlarm(2, 610, QStringLiteral("任务超时"));
    m_activeTask.captureRequestId = 0;

    if (isScanCaptureStage(m_activeTask.definition->stage)) {
        // 段扫兼跑编号时，超时需清 OCR 等待态，避免迟到回包误收尾。
        m_codeReadPending = false;
        m_codeReadSoftPending = false;
        m_codeReadCameraIp.clear();
        m_advancePathAfterTriggerRelease = false;
        // Res=6：completeScanSegmentCapture 内按 scanFailurePolicy 清理。
        completeScanSegmentCapture(6, 0, 0, protocol::AckState::Failed, false);
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::SelfCheck) {
        constexpr quint16 kFailCapture = 1u << 4;
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        return;
    }
    if (m_codeReadPending || isActiveCodeReadTrigger()) {
        finishCodeRead(2, QString(), QStringLiteral("编号识别超时。"));
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::Inspection) {
        InspectionResult timeoutResult;
        timeoutResult.resultCode = kInspectionResTimeoutNg;
        timeoutResult.message = QStringLiteral("检测任务超时");
        // 不走 finishInspection：避免 mark「已检测」并误切路径；对齐工位1超时清理策略。
        publishInspectionOutcome(timeoutResult, QStringLiteral("Trig_Inspection"));
        completeActiveTask(
            kInspectionResTimeoutNg, protocol::AckState::Failed, false);
        applyInspectionTimeoutFailurePolicy();
        return;
    }

    completeActiveTask(6, protocol::AckState::Completed, false);
}

bool StateMachine::completeActiveTask(
    quint16 resultCode,
    protocol::AckState finalAckState,
    bool dataValid)
{
    if (m_activeTask.definition == nullptr || !m_modbus) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("无法完成任务：任务定义或 Modbus 为空");
        return false;
    }

    const int ackOffset = m_activeTask.definition->ackOffset;
    const int resOffset = m_activeTask.definition->resOffset;

    auto failCompletionWrite = [this](const QString& reason) -> bool {
        qWarning(LOG_FLOW).noquote() << reason;
        enterFaultState(902, reason, false, false);
        return false;
    };

    if (resOffset == ackOffset + 1) {
        if (!m_modbus->writeRegisters(ackOffset, {
                static_cast<quint16>(finalAckState),
                resultCode,
            })) {
            return failCompletionWrite(QStringLiteral("批量写入 Ack/Res 失败"));
        }
    } else {
        sendRes(*m_activeTask.definition, resultCode);
        sendAck(*m_activeTask.definition, finalAckState);
    }

    m_timeoutTimer->stop();
    m_progress = 100;
    m_dataValid = dataValid;
    m_activeTask.completionAnnounced = true;
    m_activeTask.captureRequestId = 0;
    publishIpcStatus();

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("触发已完成") << protocol::triggerName(*m_activeTask.definition)
        << QStringLiteral(" Res=") << resultCode
        << QStringLiteral(" Ack=") << static_cast<int>(finalAckState);

    // PLC 常在采集完成前就把 Trig 拉低；若等下次写寄存器才 finalize，
    // 下次再置 1 时会因「Trig≠0」无法释放，后续触发全部被忽略。
    finalizeCompletedTaskIfTriggerReleased(m_lastCommandBlock);
    return true;
}

void StateMachine::finalizeCompletedTaskIfTriggerReleased(
    const QVector<quint16>& commandBlock,
    bool force)
{
    if (m_activeTask.definition == nullptr || !m_activeTask.completionAnnounced) {
        return;
    }

    const int trigOffset = m_activeTask.definition->trigOffset;
    if (!force && (trigOffset >= commandBlock.size() || commandBlock[trigOffset] != 0)) {
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << (force ? QStringLiteral("强制释放已完成触发：")
                  : QStringLiteral("PLC 已释放触发："))
        << protocol::triggerName(*m_activeTask.definition);

    const protocol::TriggerDefinition& definition = *m_activeTask.definition;
    if (definition.stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(0, 0, 0);
    } else if (definition.stage == protocol::Stage::TelescopicScan) {
        writeTelescopicScanResult(0, 0, 0);
    }

    if (m_modbus) {
        const int ackOffset = definition.ackOffset;
        const int resOffset = definition.resOffset;
        if (resOffset == ackOffset + 1) {
            m_modbus->writeRegisters(ackOffset, {
                static_cast<quint16>(protocol::AckState::Idle),
                0,
            });
        } else {
            sendRes(definition, 0);
            sendAck(definition, protocol::AckState::Idle);
        }
    }

    clearActiveTask();
    m_ipcState = protocol::IpcState::Ready;
    m_currentStage = protocol::Stage::Idle;
    m_progress = 0;
    setState(AppState::Ready);
    publishIpcStatus();

    if (m_advancePathAfterTriggerRelease) {
        m_advancePathAfterTriggerRelease = false;
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("Trig 已释放，执行编号段扫后的自动切路径。");
        prepareNextScanPathAfterSuccess();
    }
}

const protocol::TriggerDefinition* StateMachine::selectPendingTrigger(
    const QVector<quint16>& commandBlock,
    const QVector<quint16>& previousCommandBlock) const
{
    // 无上一拍时无法判断上升沿，不接受任何 Trig。
    if (previousCommandBlock.isEmpty()) {
        return nullptr;
    }

    for (const auto& trigger : protocol::triggerDefinitions()) {
        if (trigger.trigOffset < 0 || trigger.trigOffset >= commandBlock.size()) {
            continue;
        }
        if (trigger.trigOffset >= previousCommandBlock.size()) {
            continue;
        }
        if (commandBlock.value(trigger.trigOffset) != 1) {
            continue;
        }
        // 看门狗强制收尾后：同一 Trig 仍为 1 时忽略，直到其先回 0。
        if (m_blockTrigUntilIdleOffset > 0 &&
            trigger.trigOffset == m_blockTrigUntilIdleOffset) {
            continue;
        }

        const bool risingEdge =
            previousCommandBlock.value(trigger.trigOffset) == 0;
        const bool latchedWhileBusy =
            trigger.trigOffset >= 0 && trigger.trigOffset < 64 &&
            (g_latchedTrigRisingMask & (1ull << static_cast<unsigned>(trigger.trigOffset))) != 0;
        // 本拍 0→1，或忙碌期间已锁存且空闲后 Trig 仍保持为 1。
        if (!risingEdge && !latchedWhileBusy) {
            continue;
        }
        if (latchedWhileBusy && !risingEdge) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("补接受忙碌期间锁存的触发 ")
                << protocol::triggerName(trigger);
        }
        return &trigger;
    }
    return nullptr;
}

void StateMachine::recordModbusFailure(quint16 alarmCode, const QString& message)
{
    ++m_consecutiveModbusFailures;
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("记录 Modbus 失败")
        << m_consecutiveModbusFailures << QStringLiteral("/") << kMaxConsecutiveModbusFailures
        << message;

    if (m_consecutiveModbusFailures >= kMaxConsecutiveModbusFailures) {
        enterFaultState(alarmCode, message, true, true);
    }
}

void StateMachine::resetModbusFailureCounter()
{
    m_consecutiveModbusFailures = 0;
}

void StateMachine::enterFaultState(
    quint16 alarmCode,
    const QString& message,
    bool abortCurrentTask,
    bool notifyPlc)
{
    setAlarm(3, alarmCode, message);
    m_ipcState = protocol::IpcState::Fault;
    setState(AppState::Error);

    if (abortCurrentTask) {
        abortActiveTaskForFault(7);
    } else {
        m_timeoutTimer->stop();
        m_progress = 0;
        m_currentStage = protocol::Stage::Idle;
        publishIpcStatus();
    }

    if (!notifyPlc) {
        clearActiveTask();
        m_currentStage = protocol::Stage::Idle;
    }
}

void StateMachine::abortActiveTaskForFault(quint16 resultCode)
{
    if (m_activeTask.definition == nullptr) {
        m_timeoutTimer->stop();
        m_progress = 0;
        m_dataValid = false;
        m_currentStage = protocol::Stage::Idle;
        publishIpcStatus();
        return;
    }

    if (isScanCaptureStage(m_activeTask.definition->stage)) {
        // 与 completeScanSegmentCapture 失败路径一致：先按策略清缓存再写失败 Res。
        if (!m_activeTask.completionAnnounced && resultCode >= 5) {
            const int pathId =
                common::ConfigManager::instance() != nullptr
                    ? common::ConfigManager::instance()->activePathId()
                    : 0;
            applyScanFailurePolicy(
                pathId,
                activeScanDeviceKind(),
                m_activeTask.scanSegmentIndex,
                resultCode);
        }
        if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
            writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
        } else {
            writeTelescopicScanResult(m_activeTask.scanSegmentIndex, 0, 0);
        }
    }

    if (m_modbus && m_modbus->isConnected()) {
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        return;
    }

    m_timeoutTimer->stop();
    m_progress = 0;
    m_dataValid = false;
    m_activeTask.captureRequestId = 0;
    m_activeTask.completionAnnounced = false;
    clearActiveTask();
    m_currentStage = protocol::Stage::Idle;
    publishIpcStatus();
}

quint32 StateMachine::readTaskId(const QVector<quint16>& commandBlock) const
{
    const quint32 high = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdHigh));
    const quint32 low = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdLow));
    return (high << 16) | low;
}

quint16 StateMachine::resolveScanSegmentIndex(const QVector<quint16>& commandBlock,
                                              protocol::Stage stage) const
{
    return protocol::registers::resolveScanSegmentIndexFromBlock(commandBlock, stage);
}

}  // namespace scan_tracking::flow_control
