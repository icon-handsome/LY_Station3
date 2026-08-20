#pragma once

#include <QtCore/QString>

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking::vision {

/// 分段 LB 位姿落盘目录（与点云/BMP 同层，不再建 lb_pose 子目录）：
/// <runRoot>/path_{pathId}/{arm|telescopic}/{segmentIndex}/
QString buildSegmentLbPoseDir(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// Rt_global 文本路径：.../Path{pathId}_{Arm|Telescopic}_rt_global_{segmentIndex}.txt
QString buildSegmentLbPoseMatrixPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// 诊断文本路径：.../Path{pathId}_{Arm|Telescopic}_diagnostic_{segmentIndex}.txt
QString buildSegmentLbPoseDiagnosticPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// 将 LB 结果落盘（rt_global；diagnosticText 非空时另写 diagnostic）。
/// 仅应在 lb.invoked==true 时调用；矩阵按行主序 4×4 写出。
bool saveLbPoseResultToDisk(
    const LbPoseResult& lb,
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex,
    QString* errorMessage = nullptr);

}  // namespace scan_tracking::vision
