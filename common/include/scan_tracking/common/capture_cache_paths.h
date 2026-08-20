#pragma once

#include <QtCore/QString>

namespace scan_tracking::common {

/// 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache
QString defaultCaptureCacheRoot();

/// 配置为空时使用默认根目录，否则使用配置路径（绝对化）
QString resolveCaptureCacheRoot(const QString& configuredRoot);

/// 确保目录存在；失败返回空字符串
QString ensureDirectoryExists(const QString& directoryPath);

/// 单次运行实例下某点位目录：
/// <runRoot>/path_{pathId}/{arm|telescopic}/{pointIndex}/
QString capturePointDirectory(
    const QString& runRoot,
    int pathId,
    const QString& deviceTag,
    int pointIndex);

/// 设备标签用于文件名：arm→Arm，telescopic→Telescopic，其它首字母大写
QString captureDeviceTagForFileName(const QString& deviceTag);

/// 段产物唯一文件名：Path{pathId}_{Arm|Telescopic}_{artifact}_{pointIndex}.{ext}
/// 例：Path3_Arm_cloud_1.ply
QString buildCaptureArtifactFileName(
    int pathId,
    const QString& deviceTag,
    const QString& artifactStem,
    int pointIndex,
    const QString& extension);

/// Mech-Eye 3D 点云：<root>/mech_3d（遗留/调试缓存，主流程落盘请用 capturePointDirectory）
QString captureCacheMech3DDir(const QString& root);

/// Mech-Eye 深度纹理灰度图（非独立 2D 相机）：<root>/mech_texture
QString captureCacheMechTextureDir(const QString& root);

/// 海康 Mono 根目录：<root>/hik_mono
QString captureCacheHikMonoDir(const QString& root);

/// 海康 A/B 分目录：<root>/hik_mono/camera_a 或 camera_b（cameraTag 为 hikA / hikB）
QString captureCacheHikMonoCameraDir(const QString& root, const QString& cameraTag);

/// Orbbec Gemini 深度/点云：<root>/orbbec
QString captureCacheOrbbecDir(const QString& root);

/// 同一次分段落盘共用的时间戳：yyyy_MM_dd_HH_mm_ss_zzz
QString buildCaptureTimestamp();

/// 单次 PLC 任务采集落盘根目录：<applicationDir>/output/run_{taskId}_{timestamp}
QString buildRunCaptureRoot(quint32 taskId, const QString& timestamp = QString());

}  // namespace scan_tracking::common
