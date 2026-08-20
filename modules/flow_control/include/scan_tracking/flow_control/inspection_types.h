#pragma once

#include <QtCore/QJsonObject>
#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

namespace scan_tracking {
namespace flow_control {

struct InspectionMeasurement {
    int qualityCode = 0;
    // weld_section
    double mismatchMm = 0.0;
    double reinforcementMm = 0.0;
    double angularityMm = 0.0;
    double includedAngleDeg = 0.0;
    double leftUndercutMm = 0.0;
    double rightUndercutMm = 0.0;
    double maxUndercutMm = 0.0;
    int measuredSegmentCount = 0;
    // thickness_inner_surface
    double thicknessMm = 0.0;
    int thicknessPairCount = 0;
    int thicknessSuccessCount = 0;
    double innerDiameterMm = 0.0;
    double innerCircumferenceMm = 0.0;
    double innerRoundness = 0.0;
    int innerSurfacePairCount = 0;
    int innerSurfaceSuccessCount = 0;
    // path3 container total length (algorithm id remains length_volume for compatibility)
    double lengthMm = 0.0;
    double volumeLiters = 0.0;
    double volumeRadiusMm = 0.0;
    double fittedOuterRadiusMm = 0.0;
    double containerLeftEndPositionMm = 0.0;
    double containerRightEndPositionMm = 0.0;
    double containerIcpFitness = 0.0;
    double containerFittedRadiusMm = 0.0;
    bool containerIcpConverged = false;
    QString codeValue;
};

struct InspectionResult {
    quint16 resultCode = 0;
    quint16 ngReasonWord0 = 0;
    quint16 ngReasonWord1 = 0;
    quint16 measureItemCount = 0;
    int sourcePointCount = 0;
    int pathId = 0;
    QString pathName;
    QString algorithm;
    InspectionMeasurement measurement;
    QString message;
    /// 算法解算耗时（秒）；写入 result.txt 时保留两位小数。
    double elapsedSeconds = 0.0;
};

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement);

/// 将单条路径检测结果格式化为可追加写入 result.txt 的文本块。
QString formatInspectionResultTextBlock(const InspectionResult& result);

/// 追加写入 <runCaptureRoot>/result.txt；目录不存在时尝试创建。
/// @return 是否写入成功
bool appendInspectionResultToRunFile(
    const QString& runCaptureRoot,
    const InspectionResult& result,
    QString* errorMessage = nullptr);

}  // namespace flow_control
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::flow_control::InspectionMeasurement)
Q_DECLARE_METATYPE(scan_tracking::flow_control::InspectionResult)
