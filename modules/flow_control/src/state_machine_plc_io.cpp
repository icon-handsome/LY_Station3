#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

void StateMachine::setAlarm(quint16 level, quint16 code, const QString& message)
{
    m_alarmLevel = level;
    m_alarmCode = code;
    m_warnCode = level > 0 && level < 3 ? code : 0;
    if (!message.isEmpty()) {
        emit protocolEvent(message);
    }
}

bool StateMachine::writeIpcSafetyActionWord()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return false;
    }

    const bool written = m_modbus->writeRegisters(
        protocol::registers::kIpcSafetyActionWord,
        {m_ipcSafetyActionWord});
    if (!written) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("写入 IPC_SafetyAction_Word 失败，值=") << m_ipcSafetyActionWord;
    }
    return written;
}

bool StateMachine::reportPersonZoneAlarm(bool alarm)
{
    namespace safety = protocol::safety_bits;

    if (alarm) {
        m_ipcSafetyActionWord |= safety::kAiPersonIntrusion;
        const bool plcWritten = writeIpcSafetyActionWord();
        enterFaultState(601, QStringLiteral("监控区域检测到人员"), true, false);
        m_personZoneAlarmActive = true;
        publishIpcStatus();
        return plcWritten;
    }

    m_ipcSafetyActionWord &= ~static_cast<quint16>(safety::kAiPersonIntrusion);
    const bool plcWritten = writeIpcSafetyActionWord();

    if (m_personZoneAlarmActive) {
        setAlarm(0, 0, QString());
        m_ipcState = protocol::IpcState::Ready;
        m_currentStage = protocol::Stage::Idle;
        m_personZoneAlarmActive = false;
        setState(AppState::Ready);
    }

    publishIpcStatus();
    return plcWritten;
}

void StateMachine::writeFloatPlaceholder(int startOffset, float value)
{
    if (!m_modbus) {
        return;
    }

    if (!m_modbus->writeRegisters(startOffset, floatToCdabRegisters(value))) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入浮点占位符失败，偏移=") << startOffset;
    }
}

PoseSourceResult StateMachine::resolveLoadGraspPoseSource() const
{
    return parsePoseSource(
        "SCAN_TRACKING_LOAD_GRASP_POSE",
        QStringLiteral("load-grasp-provider"),
        {125.0f, 250.0f, 375.0f, 0.0f, 90.0f, 180.0f},
        true);
}

PoseSourceResult StateMachine::resolveUnloadCalcPoseSource() const
{
    return parsePoseSource(
        "SCAN_TRACKING_UNLOAD_CALC_POSE",
        QStringLiteral("unload-calc-provider"),
        {500.0f, 600.0f, 700.0f, 0.0f, 0.0f, 90.0f},
        true);
}

void StateMachine::writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text)
{
    if (!m_modbus) {
        return;
    }

    const QString padded = text.left(registerCount * 2).leftJustified(registerCount * 2, QLatin1Char(' '));
    QVector<quint16> values;
    values.reserve(registerCount);
    for (int i = 0; i < registerCount; ++i) {
        const QChar first = padded.at(i * 2);
        const QChar second = padded.at(i * 2 + 1);
        const quint16 packed = (static_cast<quint16>(first.unicode()) << 8) |
                               static_cast<quint16>(second.unicode() & 0xFF);
        values.push_back(packed);
    }
    if (!m_modbus->writeRegisters(startOffset, values)) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 ASCII 占位符失败，偏移=") << startOffset;
    }
}

void StateMachine::writeLoadGraspResult(const PoseSourceResult& poseSource)
{
    writeFloatPlaceholder(protocol::registers::kLoadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kLoadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kLoadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kLoadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kLoadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kLoadRz, poseSource.rz);
}

void StateMachine::writeUnloadCalcResult(const PoseSourceResult& poseSource)
{
    writeFloatPlaceholder(protocol::registers::kUnloadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kUnloadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kUnloadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kUnloadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kUnloadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kUnloadRz, poseSource.rz);
}

void StateMachine::clearInspectionResultRegisters()
{
    writeInspectionResult({});
}

bool StateMachine::writeSelfCheckFailWords(const QVector<quint16>& failWords)
{
    if (!isModbusConnected()) {
        return false;
    }

    const bool word0Written =
        m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord0, failWords);
    const bool word1Written =
        m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord1, {0});
    if (!word0Written || !word1Written) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入自检失败字失败");
    }
    return word0Written && word1Written;
}

bool StateMachine::clearScanSegmentDoneRegisters()
{
    if (!isModbusConnected()) {
        return false;
    }

    const bool clearedSegment =
        m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {0, 0, 0});
    const bool clearedTelescopic =
        m_modbus->writeRegisters(protocol::registers::kTelescopicScanDoneIndex, {0, 0, 0});
    if (!clearedSegment) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("清除扫描分段完成索引失败");
    }
    if (!clearedTelescopic) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("清除伸缩杆扫描完成索引失败");
    }
    return clearedSegment && clearedTelescopic;
}

bool StateMachine::clearIpcSafetyActionWord()
{
    if (!isModbusConnected()) {
        return false;
    }

    const bool cleared = m_modbus->writeRegisters(protocol::registers::kIpcSafetyActionWord, {0});
    if (!cleared) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("清除 IPC 安全动作字失败");
    }
    return cleared;
}

void StateMachine::writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount)
{
    if (!m_modbus) {
        return;
    }

    if (!m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {
            static_cast<quint16>(segmentIndex),
            static_cast<quint16>(imageCount),
            static_cast<quint16>(cloudFrameCount),
        })) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入扫描分段进度失败");
    }
}

void StateMachine::writeTelescopicScanResult(int segmentIndex, int imageCount, int cloudFrameCount)
{
    if (!m_modbus) {
        return;
    }

    if (!m_modbus->writeRegisters(protocol::registers::kTelescopicScanDoneIndex, {
            static_cast<quint16>(segmentIndex),
            static_cast<quint16>(imageCount),
            static_cast<quint16>(cloudFrameCount),
        })) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入伸缩杆扫描分段进度失败");
    }
}

void StateMachine::writeInspectionResult(const InspectionSummary& summary)
{
    if (!m_modbus) {
        return;
    }

    if (!m_modbus->writeRegisters(protocol::registers::kNgReasonWord0, {
            summary.ngReasonWord0,
            summary.ngReasonWord1,
            summary.measureItemCount,
        })) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入检测结果失败");
    }
}

}  // namespace scan_tracking::flow_control
