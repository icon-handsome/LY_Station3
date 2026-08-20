#pragma once

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache
QString defaultScanCacheDirectory();

/// 生成分段 PLY 绝对路径：
/// <runRoot>/path_{pathId}/{arm|telescopic}/{segmentIndex}/Path{pathId}_{Arm|Telescopic}_cloud_{segmentIndex}.ply
QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// 生成分段拼接点云 PLY（与工位一 pointcloud_stitched 对应）：
/// .../Path{pathId}_{Arm|Telescopic}_cloud_stitched_{segmentIndex}.ply
QString buildSegmentStitchedPlyPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// 将 PointCloudFrame 保存为 binary_little_endian PLY：x,y,z + uchar rgb（保留全部点含 NaN）。
/// texture 有效且像素数 >= 点数时，按点序把灰度纹理写成 R=G=B；否则 RGB 写 0。
bool savePointCloudFrameToPly(
    const PointCloudFrame& frame,
    const QString& absolutePath,
    const GrayTextureFrame* texture = nullptr);

/// 从 PLY 加载点云（仅取 xyz）；兼容旧文件：纯 xyz / xyz+法向 / xyz+rgb
bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame);

/// 释放 PointCloudFrame 中的大数组，保留 pointCount/width/height 等元数据
void releasePointCloudFrameBuffers(PointCloudFrame* frame);

/// 生成分段 Mech 纹理 PNG：
/// .../Path{pathId}_{Arm|Telescopic}_texture_{segmentIndex}.png
QString buildSegmentMechTexturePngPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// 将 GrayTextureFrame 保存为 8 位灰度 PNG
bool saveGrayTextureFrameToPng(const GrayTextureFrame& frame, const QString& absolutePath);

}  // namespace scan_tracking::mech_eye
