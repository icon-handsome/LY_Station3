#pragma once

/**
 * @file orbbec_capture_io.h
 * @brief Orbbec 深度帧与点云的落盘工具
 *
 * 提供深度图 PNG（原始 16 位 + 可视化预览 8 位）保存、点云 PLY 保存，
 * 以及采集文件路径生成等无 Qt 信号槽依赖的纯 I/O 函数。
 */

#include <QtCore/QString>

#include <cstdint>
#include <vector>

namespace scan_tracking {
namespace orbbec_gemini {

/// 深度帧只读视图，指向 SDK 提供的 Y16 深度缓冲区
struct OrbbecDepthFrameView {
    const uint16_t* data = nullptr;  // 深度像素数组，长度 width * height
    int width = 0;
    int height = 0;
    float valueScale = 1.0f;         // 原始值转毫米的缩放系数
};

/// 三维点坐标（单位与 SDK PointCloudFilter 输出一致，通常为毫米）
struct OrbbecPointView {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * @brief 将深度帧保存为原始 16 位 PNG 与 8 位预览 PNG
 *
 * 原始 PNG 直接保存 uint16 深度值；预览 PNG 按当前帧有效深度的 min/max 线性归一化到 0-255。
 * 深度值为 0 的像素视为无效，不参与 min/max 统计。
 *
 * @param frame 深度帧视图
 * @param rawPngPath 16 位原始深度 PNG 输出路径；为空则跳过保存
 * @param previewPngPath 8 位预览 PNG 输出路径；为空则跳过保存
 * @param validPixelCountOut 可选，输出非零深度像素数量
 * @return 保存成功返回 true；帧数据无效或写文件失败返回 false
 */
bool saveDepthFramePngs(
    const OrbbecDepthFrameView& frame,
    const QString& rawPngPath,
    const QString& previewPngPath,
    int* validPixelCountOut = nullptr);

/**
 * @brief 将点云保存为 ASCII 格式 PLY 文件
 *
 * 保存前会过滤无效点（非有限值或原点 (0,0,0)）。
 *
 * @param points 点云坐标数组
 * @param plyPath 输出 PLY 文件路径
 * @return 存在有效点且写文件成功时返回 true
 */
bool savePointCloudPly(
    const std::vector<OrbbecPointView>& points,
    const QString& plyPath);

/**
 * @brief 生成采集文件的基础文件名（不含扩展名）
 *
 * 格式：orbbec_req{requestId}_{timestamp}
 *
 * @param requestId 采集请求 ID
 * @param timestamp 时间戳字符串；为空时自动生成
 * @return 基础文件名
 */
QString buildOrbbecCaptureBaseName(quint64 requestId, const QString& timestamp);

/**
 * @brief 根据缓存根目录与请求信息生成三类采集文件的完整路径
 *
 * 文件保存在 captureCacheOrbbecDir(cacheRoot) 目录下：
 * - {base}_depth16.png
 * - {base}_depth_preview.png
 * - {base}_pointcloud.ply
 *
 * @param cacheRoot 采集缓存根目录
 * @param requestId 采集请求 ID
 * @param timestamp 时间戳字符串；为空时自动生成
 * @param depthRawPngPath 可选，输出原始深度 PNG 路径
 * @param depthPreviewPngPath 可选，输出预览深度 PNG 路径
 * @param pointCloudPlyPath 可选，输出点云 PLY 路径
 * @return 实际使用的 Orbbec 缓存子目录；cacheRoot 无效时返回空字符串
 */
QString buildOrbbecCapturePaths(
    const QString& cacheRoot,
    quint64 requestId,
    const QString& timestamp,
    QString* depthRawPngPath,
    QString* depthPreviewPngPath,
    QString* pointCloudPlyPath);

/// 分段落盘路径（run 根目录下 orbbec 子目录）
QString buildSegmentOrbbecDepthRawPath(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

QString buildSegmentOrbbecDepthPreviewPath(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

QString buildSegmentOrbbecPlyPath(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

}  // namespace orbbec_gemini
}  // namespace scan_tracking
