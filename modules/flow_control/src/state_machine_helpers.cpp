#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/flow_control/plc_protocol.h"

#include <QtCore/QRegExp>

#include <cstring>

namespace scan_tracking::flow_control {

Q_LOGGING_CATEGORY(LOG_FLOW, "flow_control")
Q_LOGGING_CATEGORY(LOG_ALGORITHM, "algorithm")

namespace state_machine_internal {

QString formatPlcRegisterValueForLog(int modbusIndex, quint16 rawValue)
{
    namespace regs = protocol::registers;
    if (modbusIndex == regs::kArmScanSegmentIndex ||
        modbusIndex == regs::kTelescopicScanSegmentIndex ||
        modbusIndex == regs::kScanPathId) {
        const quint16 decoded = regs::plcAnalogToUInt16(rawValue, 0);
        if (rawValue != decoded) {
            return QStringLiteral("%1 (原始PLC字=%2)").arg(decoded).arg(rawValue);
        }
        return QString::number(decoded);
    }
    if (modbusIndex == regs::kRequestTimeoutSeconds) {
        const quint16 decoded = regs::plcAnalogToUInt16(rawValue, 0);
        if (rawValue != decoded) {
            return QStringLiteral("%1s (原始PLC字=%2)").arg(decoded).arg(rawValue);
        }
        return QStringLiteral("%1").arg(decoded);
    }
    return QString::number(rawValue);
}

QString formatPlcRegisterChangeForLog(int modbusIndex, quint16 oldValue, quint16 newValue)
{
    return QStringLiteral("%1 -> %2")
        .arg(formatPlcRegisterValueForLog(modbusIndex, oldValue))
        .arg(formatPlcRegisterValueForLog(modbusIndex, newValue));
}

QVector<quint16> floatToCdabRegisters(float value)
{
    quint32 raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "Unexpected float width");
    std::memcpy(&raw, &value, sizeof(raw));

    const quint16 high = static_cast<quint16>((raw >> 16) & 0xFFFFu);
    const quint16 low = static_cast<quint16>(raw & 0xFFFFu);
    return {low, high};
}

PoseSourceResult parsePoseSource(
    const char* envName,
    const QString& sourceName,
    const std::array<float, 6>& fallback,
    bool treatMissingAsSimulated)
{
    PoseSourceResult result;
    result.available = true;
    result.sourceName = sourceName;

    const QString raw = qEnvironmentVariable(envName).trimmed();
    if (raw.isEmpty()) {
        result.success = true;
        result.message = treatMissingAsSimulated
            ? QStringLiteral("未配置外部位姿源；使用模拟回退。")
            : QStringLiteral("未配置外部位姿源。");
        result.x = fallback[0];
        result.y = fallback[1];
        result.z = fallback[2];
        result.rx = fallback[3];
        result.ry = fallback[4];
        result.rz = fallback[5];
        return result;
    }

    const auto tokens = raw.split(QRegExp(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);
    if (tokens.size() < 6) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 需要 6 个值：x,y,z,rx,ry,rz。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    bool ok = false;
    const float values[6] = {
        tokens.value(0).toFloat(&ok),
        ok ? tokens.value(1).toFloat(&ok) : 0.0f,
        ok ? tokens.value(2).toFloat(&ok) : 0.0f,
        ok ? tokens.value(3).toFloat(&ok) : 0.0f,
        ok ? tokens.value(4).toFloat(&ok) : 0.0f,
        ok ? tokens.value(5).toFloat(&ok) : 0.0f,
    };
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    result.success = true;
    result.message = QStringLiteral("从外部源 %1 加载位姿。").arg(QString::fromLatin1(envName));
    result.x = values[0];
    result.y = values[1];
    result.z = values[2];
    result.rx = values[3];
    result.ry = values[4];
    result.rz = values[5];
    return result;
}

void countBundleFrames(const vision::MultiCameraCaptureBundle& bundle, int* imageCount, int* cloudFrameCount)
{
    int images = 0;
    if (bundle.hikCameraCOk()) {
        ++images;
    }
    if (bundle.hikCameraAResult.success()) {
        ++images;
    }
    if (bundle.hikCameraBResult.success()) {
        ++images;
    }
    if (bundle.mechEyeResult.texture2D.isValid()) {
        ++images;
    }

    int clouds = 0;
    if (bundle.mechEyeResult.pointCloud.isValid()) {
        clouds = 1;
    }

    if (imageCount != nullptr) {
        *imageCount = images;
    }
    if (cloudFrameCount != nullptr) {
        *cloudFrameCount = clouds;
    }
}

}  // namespace state_machine_internal
}  // namespace scan_tracking::flow_control
