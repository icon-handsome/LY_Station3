#include "scan_tracking/orbbec_gemini/orbbec_capture_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QDir>
#include <QImage>
#include <QLoggingCategory>

#include <algorithm>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

Q_LOGGING_CATEGORY(LOG_ORBBEC_IO, "orbbec.gemini.io")

namespace scan_tracking {
namespace orbbec_gemini {

namespace {

// 过滤 SDK 输出的无效点：非有限值或原点视为无效
bool isValidPoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
           && !(x == 0.0f && y == 0.0f && z == 0.0f);
}

}  // namespace

QString buildOrbbecCaptureBaseName(quint64 requestId, const QString& timestamp)
{
    const QString ts = timestamp.trimmed().isEmpty()
                           ? scan_tracking::common::buildCaptureTimestamp()
                           : timestamp;
    return QStringLiteral("orbbec_req%1_%2").arg(requestId).arg(ts);
}

QString buildOrbbecCapturePaths(
    const QString& cacheRoot,
    quint64 requestId,
    const QString& timestamp,
    QString* depthRawPngPath,
    QString* depthPreviewPngPath,
    QString* pointCloudPlyPath)
{
    const QString baseDir = scan_tracking::common::captureCacheOrbbecDir(cacheRoot);
    if (baseDir.isEmpty()) {
        return QString();
    }

    const QString baseName = buildOrbbecCaptureBaseName(requestId, timestamp);
    if (depthRawPngPath != nullptr) {
        *depthRawPngPath = QDir(baseDir).absoluteFilePath(baseName + QStringLiteral("_depth16.png"));
    }
    if (depthPreviewPngPath != nullptr) {
        *depthPreviewPngPath =
            QDir(baseDir).absoluteFilePath(baseName + QStringLiteral("_depth_preview.png"));
    }
    if (pointCloudPlyPath != nullptr) {
        *pointCloudPlyPath = QDir(baseDir).absoluteFilePath(baseName + QStringLiteral("_pointcloud.ply"));
    }
    return baseDir;
}

QString buildSegmentOrbbecDepthRawPath(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheOrbbecDir(runRoot);
    if (baseDir.isEmpty()) {
        return QString();
    }
    const QString ts = timestamp.trimmed().isEmpty()
                           ? scan_tracking::common::buildCaptureTimestamp()
                           : timestamp;
    return QDir(baseDir).absoluteFilePath(
        QStringLiteral("segment_%1_task%2_%3_depth16.png")
            .arg(segmentIndex)
            .arg(taskId)
            .arg(ts));
}

QString buildSegmentOrbbecDepthPreviewPath(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheOrbbecDir(runRoot);
    if (baseDir.isEmpty()) {
        return QString();
    }
    const QString ts = timestamp.trimmed().isEmpty()
                           ? scan_tracking::common::buildCaptureTimestamp()
                           : timestamp;
    return QDir(baseDir).absoluteFilePath(
        QStringLiteral("segment_%1_task%2_%3_depth_preview.png")
            .arg(segmentIndex)
            .arg(taskId)
            .arg(ts));
}

QString buildSegmentOrbbecPlyPath(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheOrbbecDir(runRoot);
    if (baseDir.isEmpty()) {
        return QString();
    }
    const QString ts = timestamp.trimmed().isEmpty()
                           ? scan_tracking::common::buildCaptureTimestamp()
                           : timestamp;
    return QDir(baseDir).absoluteFilePath(
        QStringLiteral("segment_%1_task%2_%3.ply").arg(segmentIndex).arg(taskId).arg(ts));
}

bool saveDepthFramePngs(
    const OrbbecDepthFrameView& frame,
    const QString& rawPngPath,
    const QString& previewPngPath,
    int* validPixelCountOut)
{
    if (frame.data == nullptr || frame.width <= 0 || frame.height <= 0) {
        qWarning(LOG_ORBBEC_IO) << "saveDepthFramePngs: invalid frame";
        return false;
    }

    QImage rawImage(frame.width, frame.height, QImage::Format_Grayscale16);
    if (rawImage.isNull()) {
        qWarning(LOG_ORBBEC_IO) << "saveDepthFramePngs: failed to allocate raw image";
        return false;
    }

    // 单次遍历：拷贝原始深度并统计有效像素的 min/max（用于预览归一化）
    uint16_t minValue = std::numeric_limits<uint16_t>::max();
    uint16_t maxValue = 0;
    int validCount = 0;

    for (int y = 0; y < frame.height; ++y) {
        auto* scanLine = reinterpret_cast<uint16_t*>(rawImage.scanLine(y));
        for (int x = 0; x < frame.width; ++x) {
            const int index = y * frame.width + x;
            const uint16_t value = frame.data[index];
            scanLine[x] = value;
            if (value == 0) {
                continue;
            }
            ++validCount;
            minValue = std::min(minValue, value);
            maxValue = std::max(maxValue, value);
        }
    }

    if (validPixelCountOut != nullptr) {
        *validPixelCountOut = validCount;
    }

    if (!rawPngPath.trimmed().isEmpty()) {
        if (!rawImage.save(rawPngPath)) {
            qWarning(LOG_ORBBEC_IO).noquote()
                << QStringLiteral("saveDepthFramePngs: failed to save raw depth PNG:") << rawPngPath;
            return false;
        }
    }

    if (!previewPngPath.trimmed().isEmpty()) {
        QImage preview(frame.width, frame.height, QImage::Format_Grayscale8);
        if (preview.isNull()) {
            qWarning(LOG_ORBBEC_IO) << "saveDepthFramePngs: failed to allocate preview image";
            return false;
        }

        // 预览图：将有效深度线性映射到 0-255，便于人工查看
        const float scale = frame.valueScale > 0.0f ? frame.valueScale : 1.0f;
        const float minMm = static_cast<float>(minValue) * scale;
        const float maxMm = static_cast<float>(maxValue) * scale;
        const float rangeMm = std::max(maxMm - minMm, 1.0f);

        for (int y = 0; y < frame.height; ++y) {
            auto* scanLine = reinterpret_cast<unsigned char*>(preview.scanLine(y));
            for (int x = 0; x < frame.width; ++x) {
                const int index = y * frame.width + x;
                const uint16_t value = frame.data[index];
                if (value == 0) {
                    scanLine[x] = 0;
                    continue;
                }
                const float depthMm = static_cast<float>(value) * scale;
                const float normalized = (depthMm - minMm) / rangeMm;
                scanLine[x] = static_cast<unsigned char>(std::clamp(normalized, 0.0f, 1.0f) * 255.0f);
            }
        }

        if (!preview.save(previewPngPath)) {
            qWarning(LOG_ORBBEC_IO).noquote()
                << QStringLiteral("saveDepthFramePngs: failed to save preview PNG:") << previewPngPath;
            return false;
        }
    }

    qInfo(LOG_ORBBEC_IO).noquote()
        << QStringLiteral("Saved depth PNGs validPixels=%1 minRaw=%2 maxRaw=%3 scale=%4")
               .arg(validCount)
               .arg(minValue)
               .arg(maxValue)
               .arg(frame.valueScale, 0, 'f', 4);
    return true;
}

bool savePointCloudPly(
    const std::vector<OrbbecPointView>& points,
    const QString& plyPath)
{
    if (plyPath.trimmed().isEmpty()) {
        return false;
    }

    // 仅保存有效点，避免 PLY 中出现大量零点噪声
    std::vector<OrbbecPointView> validPoints;
    validPoints.reserve(points.size());
    for (const OrbbecPointView& point : points) {
        if (isValidPoint(point.x, point.y, point.z)) {
            validPoints.push_back(point);
        }
    }

    if (validPoints.empty()) {
        qWarning(LOG_ORBBEC_IO) << "savePointCloudPly: no valid points";
        return false;
    }

    std::ofstream stream(plyPath.toStdString(), std::ios::out | std::ios::binary);
    if (!stream.is_open()) {
        qWarning(LOG_ORBBEC_IO).noquote()
            << QStringLiteral("savePointCloudPly: failed to open:") << plyPath;
        return false;
    }

    stream << "ply\n";
    stream << "format ascii 1.0\n";
    stream << "element vertex " << validPoints.size() << "\n";
    stream << "property float x\n";
    stream << "property float y\n";
    stream << "property float z\n";
    stream << "end_header\n";

    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream.precision(6);
    for (const OrbbecPointView& point : validPoints) {
        stream << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }

    qInfo(LOG_ORBBEC_IO).noquote()
        << QStringLiteral("Saved point cloud PLY points=%1 path=%2")
               .arg(validPoints.size())
               .arg(plyPath);
    return true;
}

}  // namespace orbbec_gemini
}  // namespace scan_tracking
