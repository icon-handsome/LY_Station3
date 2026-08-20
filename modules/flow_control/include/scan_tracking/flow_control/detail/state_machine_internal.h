#pragma once

#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/flow_control/plc_task_host.h"
#include "scan_tracking/vision/vision_types.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QString>
#include <QtCore/QVector>

#include <array>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)
/// 测量算法编排/解算生命周期（写入 logs/algorithm_scan_tracking_*.txt）
Q_DECLARE_LOGGING_CATEGORY(LOG_ALGORITHM)

namespace state_machine_internal {

constexpr quint16 kDeviceOnlineWord0 =
    (1u << 0) | (1u << 1) | (1u << 2) | (1u << 4) | (1u << 5) | (1u << 6);

constexpr int kPollLogEveryN = 20;
constexpr int kMaxConsecutiveModbusFailures = 3;
constexpr quint16 kInspectionResTimeoutNg = 6;

QString formatPlcRegisterValueForLog(int modbusIndex, quint16 rawValue);
QString formatPlcRegisterChangeForLog(int modbusIndex, quint16 oldValue, quint16 newValue);
QVector<quint16> floatToCdabRegisters(float value);

PoseSourceResult parsePoseSource(
    const char* envName,
    const QString& sourceName,
    const std::array<float, 6>& fallback,
    bool treatMissingAsSimulated);

void countBundleFrames(const vision::MultiCameraCaptureBundle& bundle, int* imageCount, int* cloudFrameCount);

inline bool isScanCaptureStage(protocol::Stage stage)
{
    return stage == protocol::Stage::ScanSegment || stage == protocol::Stage::TelescopicScan;
}

}  // namespace state_machine_internal
}  // namespace scan_tracking::flow_control
