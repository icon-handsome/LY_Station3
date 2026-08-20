#pragma once

#include <QtCore/QString>

namespace scan_tracking {
namespace vision {

/// 与工位一对齐：在 Initialize 前补齐 GENICAM_GENTL64_PATH（CXP GenTL Producer）。
void ensureHikGenTlEnvironment();

/// 全局 MVS SDK 引用计数（MvCameraControl Initialize/Finalize）
bool acquireHikMvsSdk(QString* errorMessage = nullptr);
void releaseHikMvsSdk();

}  // namespace vision
}  // namespace scan_tracking
