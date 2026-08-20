#pragma once

// 离线 LB 位姿检查（Trig_PoseCheck）：从 dataRoot 下左右 BMP 解算 Rt。
// 目录约定与在线一致：L = 左目 = 相机 B，R = 右目 = 相机 A。

#include <array>

#include <QtCore/QString>

#include "scan_tracking/common/config_manager.h"

namespace scan_tracking::tracking {

/// 位姿校验结果，封装 LB 离线检测输出。
struct PoseCheckResult {
    bool invoked = false;
    bool success = false;
    quint16 resultCode = 7;  ///< 1=成功，其它=失败
    int inputPointCount = 0;
    double poseDeviationMm = 0.0;
    std::array<double, 16> rt = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    QString message;

    bool hasPoseMatrix() const { return success && resultCode == 1; }
};

PoseCheckResult runLegacyLbPoseCheck(const scan_tracking::common::LbPoseConfig& config);

}  // namespace scan_tracking::tracking
