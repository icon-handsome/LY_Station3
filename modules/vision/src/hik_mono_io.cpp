#include "scan_tracking/vision/hik_mono_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QLoggingCategory>

#include <cstring>

Q_LOGGING_CATEGORY(LOG_HIK_MONO_IO, "vision.hik_mono_io")

namespace scan_tracking::vision {

namespace {

constexpr quint16 kBmpFileType = 0x4D42;  // 'BM'
constexpr quint32 kBmpInfoHeaderSize = 40;
constexpr quint32 kBmpPaletteBytes = 256 * 4;
constexpr quint32 kBmpPixelDataOffset = 14 + kBmpInfoHeaderSize + kBmpPaletteBytes;

int bmpRowStride(int width)
{
    return ((width + 3) / 4) * 4;
}

}  // namespace

QString buildSegmentHikMonoPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex,
    const QString& cameraTag)
{
    const QString pointDir = scan_tracking::common::capturePointDirectory(
        configuredRoot, pathId, deviceTag, segmentIndex);
    if (pointDir.isEmpty()) {
        qWarning(LOG_HIK_MONO_IO).noquote()
            << QStringLiteral("无法创建点位目录(hik) pathId=") << pathId
            << QStringLiteral(" device=") << deviceTag
            << QStringLiteral(" point=") << segmentIndex;
        return QString();
    }

    QString fileStem = cameraTag.trimmed().toLower();
    if (fileStem.isEmpty()) {
        fileStem = QStringLiteral("hik");
    } else if (fileStem == QLatin1String("hika")) {
        fileStem = QStringLiteral("hikA");
    } else if (fileStem == QLatin1String("hikb")) {
        fileStem = QStringLiteral("hikB");
    } else if (fileStem.startsWith(QLatin1String("hikc"))) {
        fileStem = QStringLiteral("hikC");
    }

    return QDir(pointDir).absoluteFilePath(
        scan_tracking::common::buildCaptureArtifactFileName(
            pathId, deviceTag, fileStem, segmentIndex, QStringLiteral("bmp")));
}

bool saveHikMonoFrameToBmp(const HikMonoFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
        qWarning(LOG_HIK_MONO_IO) << QStringLiteral("saveHikMonoFrameToBmp：帧或路径无效");
        return false;
    }

    if (frame.stride <= 0 || frame.width <= 0 || frame.height <= 0) {
        qWarning(LOG_HIK_MONO_IO) << QStringLiteral("saveHikMonoFrameToBmp：几何尺寸无效");
        return false;
    }

    QFileInfo fileInfo(absolutePath);
    QDir().mkpath(fileInfo.absolutePath());

    const int rowStride = bmpRowStride(frame.width);
    const quint32 imageSize = static_cast<quint32>(rowStride * frame.height);
    const quint32 fileSize = kBmpPixelDataOffset + imageSize;

    QByteArray fileBuffer;
    fileBuffer.resize(static_cast<int>(fileSize));
    if (fileBuffer.size() != static_cast<int>(fileSize)) {
        qWarning(LOG_HIK_MONO_IO) << QStringLiteral("saveHikMonoFrameToBmp：分配缓冲区失败");
        return false;
    }

    auto* data = reinterpret_cast<uchar*>(fileBuffer.data());

    // BITMAPFILEHEADER
    std::memcpy(data + 0, &kBmpFileType, 2);
    std::memcpy(data + 2, &fileSize, 4);
    const quint16 reserved = 0;
    std::memcpy(data + 6, &reserved, 2);
    std::memcpy(data + 8, &reserved, 2);
    std::memcpy(data + 10, &kBmpPixelDataOffset, 4);

    // BITMAPINFOHEADER
    const qint32 width = frame.width;
    const qint32 height = frame.height;
    const quint16 planes = 1;
    const quint16 bitCount = 8;
    const quint32 compression = 0;
    const qint32 ppm = 3780;  // 96 DPI placeholder
    std::memcpy(data + 14, &kBmpInfoHeaderSize, 4);
    std::memcpy(data + 18, &width, 4);
    std::memcpy(data + 22, &height, 4);
    std::memcpy(data + 26, &planes, 2);
    std::memcpy(data + 28, &bitCount, 2);
    std::memcpy(data + 30, &compression, 4);
    std::memcpy(data + 34, &imageSize, 4);
    std::memcpy(data + 38, &ppm, 4);
    std::memcpy(data + 42, &ppm, 4);
    const quint32 colorsUsed = 256;
    const quint32 importantColors = 256;
    std::memcpy(data + 46, &colorsUsed, 4);
    std::memcpy(data + 50, &importantColors, 4);

    // 256-entry grayscale palette (BGRA)
    uchar* palette = data + 54;
    for (int index = 0; index < 256; ++index) {
        palette[index * 4 + 0] = static_cast<uchar>(index);
        palette[index * 4 + 1] = static_cast<uchar>(index);
        palette[index * 4 + 2] = static_cast<uchar>(index);
        palette[index * 4 + 3] = 0;
    }

    const auto& pixels = *frame.pixels;
    const int sourceStride = frame.stride > 0 ? frame.stride : frame.width;
    uchar* dstRows = data + kBmpPixelDataOffset;

    for (int row = 0; row < frame.height; ++row) {
        const int srcRow = frame.height - 1 - row;
        const int srcOffset = srcRow * sourceStride;
        if (srcOffset + frame.width > static_cast<int>(pixels.size())) {
            qWarning(LOG_HIK_MONO_IO) << QStringLiteral("saveHikMonoFrameToBmp：像素缓冲区越界");
            return false;
        }
        uchar* dstRow = dstRows + row * rowStride;
        std::memcpy(dstRow, pixels.data() + srcOffset, static_cast<std::size_t>(frame.width));
        if (rowStride > frame.width) {
            std::memset(dstRow + frame.width, 0, static_cast<std::size_t>(rowStride - frame.width));
        }
    }

    QFile file(absolutePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning(LOG_HIK_MONO_IO).noquote()
            << QStringLiteral("saveHikMonoFrameToBmp：无法打开") << absolutePath;
        return false;
    }
    if (file.write(fileBuffer) != fileBuffer.size()) {
        qWarning(LOG_HIK_MONO_IO) << QStringLiteral("saveHikMonoFrameToBmp：写入失败");
        return false;
    }
    file.close();

    qInfo(LOG_HIK_MONO_IO).noquote()
        << QStringLiteral("BMP 已保存：") << absolutePath
        << QStringLiteral(" size=") << frame.width << QStringLiteral("x") << frame.height;

    return true;
}

void releaseHikMonoFrameBuffers(HikMonoFrame* frame)
{
    if (frame == nullptr) {
        return;
    }
    frame->pixels.reset();
}

}  // namespace scan_tracking::vision
