#include "scan_tracking/mech_eye/point_cloud_io.h"

#include "scan_tracking/common/capture_cache_paths.h"
#include <QByteArray>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QLoggingCategory>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <new>
#include <vector>

Q_LOGGING_CATEGORY(LOG_POINT_CLOUD_IO, "mech_eye.point_cloud_io")

namespace scan_tracking::mech_eye {

namespace {

enum class PlyFormat {
    Unknown,
    Ascii,
    BinaryLittleEndian,
};

struct PlyHeader {
    PlyFormat format = PlyFormat::Unknown;
    int vertexCount = 0;
    bool hasNormals = false;
    bool hasRgb = false;
    int headerEndOffset = 0;
    /// 每个顶点二进制字节数（仅 binary）；未知时为 0
    int bytesPerVertex = 0;
};

bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

int propertyByteSize(const QString& typeToken)
{
    if (typeToken == QLatin1String("float") || typeToken == QLatin1String("float32")
        || typeToken == QLatin1String("int") || typeToken == QLatin1String("int32")
        || typeToken == QLatin1String("uint") || typeToken == QLatin1String("uint32")) {
        return 4;
    }
    if (typeToken == QLatin1String("double") || typeToken == QLatin1String("float64")) {
        return 8;
    }
    if (typeToken == QLatin1String("uchar") || typeToken == QLatin1String("uint8")
        || typeToken == QLatin1String("char") || typeToken == QLatin1String("int8")) {
        return 1;
    }
    if (typeToken == QLatin1String("ushort") || typeToken == QLatin1String("uint16")
        || typeToken == QLatin1String("short") || typeToken == QLatin1String("int16")) {
        return 2;
    }
    return 0;
}

bool parsePlyHeader(QIODevice* device, PlyHeader* header, QString* errorMessage)
{
    if (header == nullptr || device == nullptr) {
        return false;
    }

    const QByteArray firstLine = device->readLine();
    if (firstLine.trimmed() != "ply") {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("缺少 PLY 头");
        }
        return false;
    }

    PlyHeader parsed;
    bool inHeader = true;
    bool inVertexElement = false;
    while (inHeader && !device->atEnd()) {
        const QByteArray rawLine = device->readLine();
        const QString line = QString::fromUtf8(rawLine).trimmed();
        if (line.startsWith(QStringLiteral("format "))) {
            if (line.contains(QStringLiteral("ascii"))) {
                parsed.format = PlyFormat::Ascii;
            } else if (line.contains(QStringLiteral("binary_little_endian"))) {
                parsed.format = PlyFormat::BinaryLittleEndian;
            }
        } else if (line.startsWith(QStringLiteral("element "))) {
            inVertexElement = line.startsWith(QStringLiteral("element vertex"));
            if (inVertexElement) {
                parsed.vertexCount = line.section(QLatin1Char(' '), 2).toInt();
            }
        } else if (inVertexElement && line.startsWith(QStringLiteral("property "))) {
            const QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (tokens.size() >= 3) {
                const int size = propertyByteSize(tokens[1]);
                if (size > 0) {
                    parsed.bytesPerVertex += size;
                }
                if (tokens[2] == QLatin1String("nx")) {
                    parsed.hasNormals = true;
                } else if (tokens[2] == QLatin1String("red") || tokens[2] == QLatin1String("r")) {
                    parsed.hasRgb = true;
                }
            }
        } else if (line == QStringLiteral("end_header")) {
            inHeader = false;
            parsed.headerEndOffset = static_cast<int>(device->pos());
        }
    }

    if (parsed.format == PlyFormat::Unknown || parsed.vertexCount <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("PLY 头无效");
        }
        return false;
    }

    // 兼容旧写法：未统计到 property 时按 xyz / xyz+nxny nz 回退
    if (parsed.bytesPerVertex <= 0) {
        parsed.bytesPerVertex = parsed.hasNormals ? 24 : 12;
        if (parsed.hasRgb) {
            parsed.bytesPerVertex += 3;
        }
    }

    *header = parsed;
    return true;
}

}  // namespace

QString defaultScanCacheDirectory()
{
    return scan_tracking::common::defaultCaptureCacheRoot();
}

QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString pointDir = scan_tracking::common::capturePointDirectory(
        configuredRoot, pathId, deviceTag, segmentIndex);
    if (pointDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("无法创建点位目录 pathId=") << pathId
            << QStringLiteral(" device=") << deviceTag
            << QStringLiteral(" point=") << segmentIndex;
        return QString();
    }
    return QDir(pointDir).absoluteFilePath(
        scan_tracking::common::buildCaptureArtifactFileName(
            pathId, deviceTag, QStringLiteral("cloud"), segmentIndex, QStringLiteral("ply")));
}

QString buildSegmentStitchedPlyPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString pointDir = scan_tracking::common::capturePointDirectory(
        configuredRoot, pathId, deviceTag, segmentIndex);
    if (pointDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("无法创建点位目录(stitched) pathId=") << pathId
            << QStringLiteral(" device=") << deviceTag
            << QStringLiteral(" point=") << segmentIndex;
        return QString();
    }
    return QDir(pointDir).absoluteFilePath(
        scan_tracking::common::buildCaptureArtifactFileName(
            pathId,
            deviceTag,
            QStringLiteral("cloud_stitched"),
            segmentIndex,
            QStringLiteral("ply")));
}

bool savePointCloudFrameToPly(
    const PointCloudFrame& frame,
    const QString& absolutePath,
    const GrayTextureFrame* texture)
{
    try {
        if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
            qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：帧或路径无效");
            return false;
        }
        if (frame.pointsXYZ == nullptr) {
            qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：pointsXYZ 为空");
            return false;
        }

        // 持有 shared_ptr 本地副本，写盘期间缓冲不会因调用方 reset 而释放。
        // 不与 pointCloudAlgorithmMutex 交叉持有：变换只读输入并另分配输出，
        // 写盘只读缓冲；长 I/O 占锁会卡住 LB 拼接与主路径。
        const std::shared_ptr<std::vector<float>> pointsKeep = frame.pointsXYZ;
        const std::shared_ptr<std::vector<uint8_t>> textureKeep =
            (texture != nullptr && texture->isValid()) ? texture->pixels : nullptr;
        if (!pointsKeep) {
            qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：pointsXYZ 为空");
            return false;
        }

        const auto& points = *pointsKeep;
        const int pointCount = frame.pointCount;
        const int availablePointCount = static_cast<int>(points.size() / 3);
        const int count = std::min(pointCount, availablePointCount);
        if (count <= 0) {
            qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：无点可写");
            return false;
        }

        const uint8_t* texturePixels = nullptr;
        if (textureKeep) {
            const int texturePixelCount = static_cast<int>(textureKeep->size());
            if (texturePixelCount >= count) {
                texturePixels = textureKeep->data();
            } else {
                qWarning(LOG_POINT_CLOUD_IO).noquote()
                    << QStringLiteral("savePointCloudFrameToPly：纹理像素不足，RGB 将写 0")
                    << QStringLiteral(" texturePixels=") << texturePixelCount
                    << QStringLiteral(" points=") << count;
            }
        }

        QFileInfo fileInfo(absolutePath);
        if (!QDir().mkpath(fileInfo.absolutePath())) {
            qWarning(LOG_POINT_CLOUD_IO).noquote()
                << QStringLiteral("savePointCloudFrameToPly：无法创建目录")
                << fileInfo.absolutePath();
            return false;
        }

        // 用 QFile 写，避免 ofstream + toStdString 在非 ASCII 路径上出问题。
        QFile file(absolutePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            qWarning(LOG_POINT_CLOUD_IO).noquote()
                << QStringLiteral("savePointCloudFrameToPly：无法打开") << absolutePath;
            return false;
        }

        {
            QByteArray header;
            header.reserve(256);
            header += "ply\n";
            header += "format binary_little_endian 1.0\n";
            header += "element vertex ";
            header += QByteArray::number(count);
            header += "\n";
            header += "property float x\n";
            header += "property float y\n";
            header += "property float z\n";
            header += "property uchar red\n";
            header += "property uchar green\n";
            header += "property uchar blue\n";
            header += "end_header\n";
            if (file.write(header) != header.size()) {
                qWarning(LOG_POINT_CLOUD_IO).noquote()
                    << QStringLiteral("savePointCloudFrameToPly：写头失败") << absolutePath;
                file.close();
                QFile::remove(absolutePath);
                return false;
            }
        }

        // 按顶点缓冲写出，降低单次系统调用次数；失败则删半成品避免下次误读。
        constexpr int kVertsPerChunk = 8192;
        QByteArray chunk;
        chunk.resize(kVertsPerChunk * (3 * static_cast<int>(sizeof(float)) + 3));

        int written = 0;
        while (written < count) {
            const int n = std::min(kVertsPerChunk, count - written);
            char* dst = chunk.data();
            for (int i = 0; i < n; ++i) {
                const int index = written + i;
                const auto base = static_cast<std::size_t>(index * 3);
                float xyz[3] = {points[base], points[base + 1], points[base + 2]};
                std::memcpy(dst, xyz, sizeof(xyz));
                dst += sizeof(xyz);
                const uint8_t gray =
                    texturePixels != nullptr ? texturePixels[static_cast<std::size_t>(index)]
                                             : static_cast<uint8_t>(0);
                dst[0] = static_cast<char>(gray);
                dst[1] = static_cast<char>(gray);
                dst[2] = static_cast<char>(gray);
                dst += 3;
            }
            const qint64 bytes = static_cast<qint64>(n) * (3 * static_cast<qint64>(sizeof(float)) + 3);
            if (file.write(chunk.constData(), bytes) != bytes) {
                qWarning(LOG_POINT_CLOUD_IO).noquote()
                    << QStringLiteral("savePointCloudFrameToPly：写体失败") << absolutePath
                    << QStringLiteral(" at=") << written;
                file.close();
                QFile::remove(absolutePath);
                return false;
            }
            written += n;
        }

        file.flush();
        file.close();
        if (file.error() != QFile::NoError) {
            qWarning(LOG_POINT_CLOUD_IO).noquote()
                << QStringLiteral("savePointCloudFrameToPly：关闭异常") << absolutePath
                << file.errorString();
            QFile::remove(absolutePath);
            return false;
        }

        qInfo(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("PLY(binary xyz+rgb) 已保存：") << absolutePath
            << QStringLiteral(" points=") << count
            << QStringLiteral(" textured=") << (texturePixels != nullptr);
        return true;
    } catch (const std::bad_alloc&) {
        qCritical(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("savePointCloudFrameToPly：内存不足") << absolutePath;
        QFile::remove(absolutePath);
        return false;
    } catch (const std::exception& ex) {
        qCritical(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("savePointCloudFrameToPly：异常") << absolutePath
            << QString::fromUtf8(ex.what());
        QFile::remove(absolutePath);
        return false;
    } catch (...) {
        qCritical(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("savePointCloudFrameToPly：未知异常") << absolutePath;
        QFile::remove(absolutePath);
        return false;
    }
}

bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame)
{
    if (outFrame == nullptr || absolutePath.trimmed().isEmpty()) {
        return false;
    }

    QFile file(absolutePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：无法打开") << absolutePath;
        return false;
    }

    PlyHeader header;
    QString headerError;
    if (!parsePlyHeader(&file, &header, &headerError)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：") << headerError << absolutePath;
        return false;
    }

    auto points = std::make_shared<std::vector<float>>();
    auto normals = std::make_shared<std::vector<float>>();
    points->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    if (header.hasNormals) {
        normals->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    }

    if (header.format == PlyFormat::BinaryLittleEndian) {
        const qint64 bytesNeeded =
            static_cast<qint64>(header.vertexCount) * header.bytesPerVertex;
        const QByteArray body = file.read(bytesNeeded);
        if (body.size() != bytesNeeded) {
            qWarning(LOG_POINT_CLOUD_IO).noquote()
                << QStringLiteral("loadPointCloudFrameFromPly：二进制体长度不足") << absolutePath;
            return false;
        }

        for (int index = 0; index < header.vertexCount; ++index) {
            const auto offset = static_cast<std::size_t>(index * header.bytesPerVertex);
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::memcpy(&x, body.data() + offset, sizeof(float));
            std::memcpy(&y, body.data() + offset + sizeof(float), sizeof(float));
            std::memcpy(&z, body.data() + offset + 2 * sizeof(float), sizeof(float));

            points->push_back(x);
            points->push_back(y);
            points->push_back(z);

            if (header.hasNormals && header.bytesPerVertex >= 24) {
                float nx = 0.0f;
                float ny = 0.0f;
                float nz = 1.0f;
                std::memcpy(&nx, body.data() + offset + 3 * sizeof(float), sizeof(float));
                std::memcpy(&ny, body.data() + offset + 4 * sizeof(float), sizeof(float));
                std::memcpy(&nz, body.data() + offset + 5 * sizeof(float), sizeof(float));
                normals->push_back(nx);
                normals->push_back(ny);
                normals->push_back(nz);
            }
        }
    } else {
        int loaded = 0;
        while (!file.atEnd() && loaded < header.vertexCount) {
            const QByteArray rawLine = file.readLine();
            const QString line = QString::fromUtf8(rawLine).trimmed();
            if (line.isEmpty()) {
                continue;
            }

            const QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (tokens.size() < 3) {
                continue;
            }

            const float x = tokens[0].toFloat();
            const float y = tokens[1].toFloat();
            const float z = tokens[2].toFloat();
            if (!isFinitePoint(x, y, z)) {
                continue;
            }

            points->push_back(x);
            points->push_back(y);
            points->push_back(z);

            if (header.hasNormals && tokens.size() >= 6) {
                normals->push_back(tokens[3].toFloat());
                normals->push_back(tokens[4].toFloat());
                normals->push_back(tokens[5].toFloat());
            } else if (header.hasNormals) {
                normals->push_back(0.0f);
                normals->push_back(0.0f);
                normals->push_back(1.0f);
            }

            ++loaded;
        }
    }

    file.close();

    if (points->empty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：无有效点") << absolutePath;
        return false;
    }

    const int pointCount = static_cast<int>(points->size() / 3);
    outFrame->pointsXYZ = std::move(points);
    if (header.hasNormals && static_cast<int>(normals->size()) == pointCount * 3) {
        outFrame->normalsXYZ = std::move(normals);
    } else {
        outFrame->normalsXYZ.reset();
    }
    outFrame->pointCount = pointCount;
    outFrame->width = pointCount;
    outFrame->height = 1;

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("PLY 已加载：") << absolutePath
        << QStringLiteral(" pointCount=") << pointCount
        << QStringLiteral(" hasRgb=") << header.hasRgb
        << QStringLiteral(" hasNormals=") << outFrame->hasNormals();

    return true;
}

void releasePointCloudFrameBuffers(PointCloudFrame* frame)
{
    if (frame == nullptr) {
        return;
    }
    frame->pointsXYZ.reset();
    frame->normalsXYZ.reset();
}

QString buildSegmentMechTexturePngPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString pointDir = scan_tracking::common::capturePointDirectory(
        configuredRoot, pathId, deviceTag, segmentIndex);
    if (pointDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("无法创建点位目录(texture) pathId=") << pathId
            << QStringLiteral(" device=") << deviceTag
            << QStringLiteral(" point=") << segmentIndex;
        return QString();
    }
    return QDir(pointDir).absoluteFilePath(
        scan_tracking::common::buildCaptureArtifactFileName(
            pathId, deviceTag, QStringLiteral("texture"), segmentIndex, QStringLiteral("png")));
}

bool saveGrayTextureFrameToPng(const GrayTextureFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty() || frame.pixels == nullptr) {
        return false;
    }

    const auto& pixels = *frame.pixels;
    const auto expected = static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    if (frame.width <= 0 || frame.height <= 0 || pixels.size() < expected) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("saveGrayTextureFrameToPng：像素尺寸不匹配")
            << frame.width << QLatin1Char('x') << frame.height
            << QStringLiteral(" pixels=") << static_cast<qint64>(pixels.size());
        return false;
    }

    QFileInfo fileInfo(absolutePath);
    QDir().mkpath(fileInfo.absolutePath());

    QImage image(frame.width, frame.height, QImage::Format_Grayscale8);
    if (image.isNull()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("saveGrayTextureFrameToPng：无法分配 QImage")
            << frame.width << QLatin1Char('x') << frame.height;
        return false;
    }

    for (int row = 0; row < frame.height; ++row) {
        auto* scanLine = image.scanLine(row);
        if (scanLine == nullptr) {
            qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("saveGrayTextureFrameToPng：scanLine 为空");
            return false;
        }
        const auto offset = static_cast<std::size_t>(row * frame.width);
        std::memcpy(scanLine, pixels.data() + offset, static_cast<std::size_t>(frame.width));
    }

    if (!image.save(absolutePath, "PNG")) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << QStringLiteral("saveGrayTextureFrameToPng 失败：") << absolutePath;
        return false;
    }

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("Mech 纹理 PNG 已保存：") << absolutePath << frame.width << QStringLiteral("x") << frame.height;
    return true;
}

}  // namespace scan_tracking::mech_eye
