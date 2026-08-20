#pragma once

#include <QtCore/QString>

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking::vision {

/// 生成分段海康 Mono BMP：
/// .../Path{pathId}_{Arm|Telescopic}_{hikA|hikB|hikC}_{segmentIndex}.bmp
QString buildSegmentHikMonoPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex,
    const QString& cameraTag);

/// 将 Mono8 帧保存为 8 位灰度 BMP
bool saveHikMonoFrameToBmp(const HikMonoFrame& frame, const QString& absolutePath);

/// 释放像素缓冲，保留 width/height 等元数据
void releaseHikMonoFrameBuffers(HikMonoFrame* frame);

}  // namespace scan_tracking::vision
