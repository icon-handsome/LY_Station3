#include "scan_tracking/orbbec_gemini/orbbec_gemini_worker.h"

#include "scan_tracking/orbbec_gemini/orbbec_gemini_types.h"
#include "scan_tracking/orbbec_gemini/orbbec_capture_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>
#include <QtCore/QString>

#include <iomanip>
#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

#include <libobsensor/ObSensor.hpp>

Q_LOGGING_CATEGORY(LOG_ORBBEC_GEMINI, "orbbec.gemini")

namespace scan_tracking {
namespace orbbec_gemini {

namespace {

QString logPrefix()
{
    return QStringLiteral("[OrbbecGemini]");
}

OrbbecGeminiDeviceSummary summaryFromDeviceInfo(
    int index,
    const std::shared_ptr<ob::DeviceInfo>& info)
{
    OrbbecGeminiDeviceSummary summary;
    summary.index = index;
    if (info == nullptr) {
        return summary;
    }

    summary.name = QString::fromStdString(info->getName());
    summary.serialNumber = QString::fromStdString(info->getSerialNumber());
    summary.firmwareVersion = QString::fromStdString(info->getFirmwareVersion());
    summary.connectionType = QString::fromStdString(info->getConnectionType());
    summary.pid = static_cast<quint16>(info->getPid());
    summary.vid = static_cast<quint16>(info->getVid());
    summary.uid = QString::fromStdString(info->getUid());
    return summary;
}

QString formatDeviceSummaryLine(const OrbbecGeminiDeviceSummary& summary)
{
    std::ostringstream stream;
    stream << "  [" << summary.index << "] name=" << summary.name.toStdString()
           << " SN=" << summary.serialNumber.toStdString()
           << " firmware=" << summary.firmwareVersion.toStdString()
           << " connection=" << summary.connectionType.toStdString();
    return QString::fromStdString(stream.str());
}

QString formatOpenedDeviceLine(const OrbbecGeminiDeviceSummary& summary)
{
    std::ostringstream stream;
    stream << "Opened device: name=" << summary.name.toStdString()
           << " SN=" << summary.serialNumber.toStdString()
           << " firmware=" << summary.firmwareVersion.toStdString()
           << " pid=0x" << std::hex << std::setw(4) << std::setfill('0') << summary.pid
           << " vid=0x" << std::setw(4) << summary.vid << std::dec
           << " uid=" << summary.uid.toStdString()
           << " connection=" << summary.connectionType.toStdString();
    return QString::fromStdString(stream.str());
}

int positiveOrDefault(int value, int fallback)
{
    return value > 0 ? value : fallback;
}

// 轮询等待包含深度帧的 FrameSet，避免单次 waitForFrameset 阻塞过久
std::shared_ptr<ob::FrameSet> waitForDepthFrameSet(
    ob::Pipeline& pipeline,
    int timeoutMs,
    int pollIntervalMs = 100)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        const int remainingMs = timeoutMs - static_cast<int>(timer.elapsed());
        const int waitMs = std::min(pollIntervalMs, std::max(remainingMs, 1));
        auto frameSet = pipeline.waitForFrameset(waitMs);
        if (frameSet == nullptr) {
            continue;
        }
        if (frameSet->getFrame(OB_FRAME_DEPTH) != nullptr) {
            return frameSet;
        }
    }
    return nullptr;
}

// 丢弃启动后前几帧，等待自动曝光/深度稳定
void discardWarmupFrames(ob::Pipeline& pipeline, int frameCount)
{
    for (int index = 0; index < frameCount; ++index) {
        pipeline.waitForFrameset(100);
    }
}

// 使用 SDK PointCloudFilter 将深度帧转换为 OBPoint 数组
bool extractPointCloud(
    ob::Pipeline& pipeline,
    const std::shared_ptr<ob::FrameSet>& frameSet,
    std::vector<OrbbecPointView>* pointsOut)
{
    if (pointsOut == nullptr || frameSet == nullptr) {
        return false;
    }

    ob::PointCloudFilter pointCloudFilter;
    pointCloudFilter.setCreatePointFormat(OB_FORMAT_POINT);
    try {
        const OBCameraParam cameraParam = pipeline.getCameraParam();
        pointCloudFilter.setCameraParam(cameraParam);
    } catch (const ob::Error& error) {
        qWarning(LOG_ORBBEC_GEMINI).noquote()
            << QStringLiteral("%1 PointCloudFilter setCameraParam failed: %2")
                   .arg(logPrefix(), QString::fromStdString(error.what()));
    } catch (const std::exception& error) {
        qWarning(LOG_ORBBEC_GEMINI).noquote()
            << QStringLiteral("%1 PointCloudFilter setCameraParam failed: %2")
                   .arg(logPrefix(), QString::fromUtf8(error.what()));
    }

    const std::shared_ptr<ob::Frame> pointCloudFrame = pointCloudFilter.process(frameSet);
    if (pointCloudFrame == nullptr || pointCloudFrame->data() == nullptr) {
        return false;
    }

    const int pointCount = static_cast<int>(pointCloudFrame->dataSize() / sizeof(OBPoint));
    if (pointCount <= 0) {
        return false;
    }

    const auto* rawPoints = reinterpret_cast<const OBPoint*>(pointCloudFrame->data());
    pointsOut->reserve(static_cast<std::size_t>(pointCount));
    for (int index = 0; index < pointCount; ++index) {
        pointsOut->push_back(
            OrbbecPointView{rawPoints[index].x, rawPoints[index].y, rawPoints[index].z});
    }
    return !pointsOut->empty();
}

}  // namespace

// PIMPL：隔离 libobsensor 头文件，避免污染 worker 头文件的编译依赖
class OrbbecGeminiWorker::Impl {
public:
    std::unique_ptr<ob::Context> context;
    std::shared_ptr<ob::Device> device;
    std::unique_ptr<ob::Pipeline> pipeline;
    bool streamsStarted = false;
};

OrbbecGeminiWorker::OrbbecGeminiWorker(QObject* parent)
    : QObject(parent)
    , m_impl(new Impl())
{
}

OrbbecGeminiWorker::~OrbbecGeminiWorker()
{
    stopWorker();
    delete m_impl;
    m_impl = nullptr;
}

void OrbbecGeminiWorker::startWorker(const OrbbecGeminiOpenConfig& config)
{
    // 重入时先释放旧 pipeline，保证 SDK 对象生命周期清晰
    stopWorker();

    m_config = config;
    m_stopping = false;
    m_openedDevice = {};

    emit logMessage(QStringLiteral("%1 Starting worker...").arg(logPrefix()));
    emit stateChanged(OrbbecGeminiRuntimeState::Enumerating, QStringLiteral("Enumerating devices"));

    QVector<OrbbecGeminiDeviceSummary> summaries;

    try {
        m_impl->context = std::make_unique<ob::Context>();
        const std::shared_ptr<ob::DeviceList> deviceList = m_impl->context->queryDeviceList();
        const uint32_t count = deviceList->getCount();

        if (count == 0) {
            emit logMessage(QStringLiteral("%1 No device found").arg(logPrefix()));
            emit enumerateFinished(summaries);
            emit openFinished(false, OrbbecGeminiDeviceSummary{}, QStringLiteral("No Orbbec device connected"));
            emit stateChanged(OrbbecGeminiRuntimeState::Failed, QStringLiteral("No device found"));
            return;
        }

        emit logMessage(
            QStringLiteral("%1 Enumerated %2 device(s)").arg(logPrefix()).arg(count));

        for (uint32_t index = 0; index < count; ++index) {
            try {
                const std::shared_ptr<ob::Device> probeDevice = deviceList->getDevice(index);
                const std::shared_ptr<ob::DeviceInfo> info = probeDevice->getDeviceInfo();
                const OrbbecGeminiDeviceSummary summary =
                    summaryFromDeviceInfo(static_cast<int>(index), info);
                summaries.push_back(summary);
                emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), formatDeviceSummaryLine(summary)));
            } catch (const ob::Error& error) {
                emit logMessage(
                    QStringLiteral("%1 Failed to probe device index=%2: %3")
                        .arg(logPrefix())
                        .arg(index)
                        .arg(QString::fromStdString(error.what())));
            }
        }

        emit enumerateFinished(summaries);
        emit stateChanged(OrbbecGeminiRuntimeState::Opening, QStringLiteral("Opening device"));

        // 优先按序列号打开，便于多相机场景下精确定位设备
        const QString trimmedSerial = config.serial.trimmed();
        int openedIndex = config.deviceIndex;
        if (!trimmedSerial.isEmpty()) {
            emit logMessage(
                QStringLiteral("%1 Opening device by serial=%2")
                    .arg(logPrefix(), trimmedSerial));
            const QByteArray serialBytes = trimmedSerial.toUtf8();
            m_impl->device = deviceList->getDeviceBySN(serialBytes.constData());
            for (const OrbbecGeminiDeviceSummary& summary : summaries) {
                if (summary.serialNumber == trimmedSerial) {
                    openedIndex = summary.index;
                    break;
                }
            }
        } else {
            emit logMessage(
                QStringLiteral("%1 Opening device by index=%2")
                    .arg(logPrefix())
                    .arg(config.deviceIndex));
            if (config.deviceIndex < 0
                || static_cast<uint32_t>(config.deviceIndex) >= deviceList->getCount()) {
                throw std::runtime_error(
                    "Device index out of range: "
                    + std::to_string(config.deviceIndex));
            }
            openedIndex = config.deviceIndex;
            m_impl->device = deviceList->getDevice(static_cast<uint32_t>(config.deviceIndex));
        }

        m_openedDevice = summaryFromDeviceInfo(openedIndex, m_impl->device->getDeviceInfo());
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), formatOpenedDeviceLine(m_openedDevice)));

        m_impl->pipeline = std::make_unique<ob::Pipeline>(m_impl->device);
        const std::shared_ptr<ob::Config> streamConfig = std::make_shared<ob::Config>();
        const int depthWidth = positiveOrDefault(config.depthWidth, 640);
        const int depthHeight = positiveOrDefault(config.depthHeight, 480);
        const int fps = positiveOrDefault(config.fps, 15);
        // 深度流使用 Y16 格式；彩色流可选，当前采集流程不依赖彩色帧
        streamConfig->enableVideoStream(
            OB_STREAM_DEPTH,
            depthWidth,
            depthHeight,
            fps,
            OB_FORMAT_Y16);
        if (config.enableColorStream) {
            streamConfig->enableVideoStream(
                OB_STREAM_COLOR,
                depthWidth,
                depthHeight,
                fps,
                OB_FORMAT_RGB);
        }

        emit logMessage(
            QStringLiteral("%1 Starting depth stream %2x%3@%4fps")
                .arg(logPrefix())
                .arg(depthWidth)
                .arg(depthHeight)
                .arg(fps));
        m_impl->pipeline->start(streamConfig);
        m_impl->streamsStarted = true;

        const int warmupCount = std::max(config.warmupFrameCount, 0);
        if (warmupCount > 0) {
            emit logMessage(
                QStringLiteral("%1 Discarding %2 warmup frame(s)").arg(logPrefix()).arg(warmupCount));
            discardWarmupFrames(*m_impl->pipeline, warmupCount);
        }

        emit logMessage(QStringLiteral("%1 Ready (depth stream started)").arg(logPrefix()));
        emit stateChanged(OrbbecGeminiRuntimeState::Ready, QStringLiteral("Device opened"));
        emit openFinished(true, m_openedDevice, {});
    } catch (const ob::Error& error) {
        const QString message = QString::fromStdString(error.what());
        emit logMessage(QStringLiteral("%1 Open failed: %2").arg(logPrefix(), message));
        emit openFinished(false, OrbbecGeminiDeviceSummary{}, message);
        emit stateChanged(OrbbecGeminiRuntimeState::Failed, message);
        if (m_impl->pipeline != nullptr && m_impl->streamsStarted) {
            try {
                m_impl->pipeline->stop();
            } catch (...) {
            }
        }
        m_impl->pipeline.reset();
        m_impl->streamsStarted = false;
        m_impl->device.reset();
    } catch (const std::exception& error) {
        const QString message = QString::fromUtf8(error.what());
        emit logMessage(QStringLiteral("%1 Open failed: %2").arg(logPrefix(), message));
        emit openFinished(false, OrbbecGeminiDeviceSummary{}, message);
        emit stateChanged(OrbbecGeminiRuntimeState::Failed, message);
        if (m_impl->pipeline != nullptr && m_impl->streamsStarted) {
            try {
                m_impl->pipeline->stop();
            } catch (...) {
            }
        }
        m_impl->pipeline.reset();
        m_impl->streamsStarted = false;
        m_impl->device.reset();
    }
}

void OrbbecGeminiWorker::stopWorker()
{
    m_stopping = true;

    if (m_impl->pipeline != nullptr && m_impl->streamsStarted) {
        try {
            m_impl->pipeline->stop();
        } catch (const ob::Error& error) {
            emit logMessage(
                QStringLiteral("%1 Pipeline stop failed: %2")
                    .arg(logPrefix(), QString::fromStdString(error.what())));
        } catch (const std::exception& error) {
            emit logMessage(
                QStringLiteral("%1 Pipeline stop failed: %2")
                    .arg(logPrefix(), QString::fromUtf8(error.what())));
        }
    }

    m_impl->pipeline.reset();
    m_impl->streamsStarted = false;
    m_impl->device.reset();
    m_impl->context.reset();
    m_openedDevice = {};

    emit stateChanged(OrbbecGeminiRuntimeState::Stopped, QStringLiteral("Worker stopped"));
}

void OrbbecGeminiWorker::performCapture(const OrbbecCaptureRequest& request)
{
    OrbbecCaptureResult result;
    result.requestId = request.requestId;
    result.deviceInfo = m_openedDevice;

    if (m_stopping) {
        result.errorCode = OrbbecCaptureErrorCode::NotReady;
        result.errorMessage = QStringLiteral("Worker is stopping");
        emit captureFinished(result);
        return;
    }

    if (m_impl->pipeline == nullptr || !m_impl->streamsStarted) {
        result.errorCode = OrbbecCaptureErrorCode::NotReady;
        result.errorMessage = QStringLiteral("Depth stream is not started");
        emit captureFinished(result);
        return;
    }

    emit stateChanged(OrbbecGeminiRuntimeState::Capturing, QStringLiteral("Capturing frame"));
    QElapsedTimer timer;
    timer.start();

    const int timeoutMs = positiveOrDefault(request.timeoutMs, m_config.captureTimeoutMs);
    // 请求级与配置级均需允许落盘才会写文件
    const bool saveToDisk = request.saveToDisk && m_config.saveCaptureToDisk;

    try {
        auto frameSet = waitForDepthFrameSet(*m_impl->pipeline, timeoutMs);
        if (frameSet == nullptr) {
            result.errorCode = OrbbecCaptureErrorCode::Timeout;
            result.errorMessage =
                QStringLiteral("Timed out waiting for depth frame (%1 ms)").arg(timeoutMs);
            result.captureDurationMs = timer.elapsed();
            emit captureFinished(result);
            emit stateChanged(OrbbecGeminiRuntimeState::Ready, QStringLiteral("Capture timed out"));
            return;
        }

        const std::shared_ptr<ob::Frame> depthFrameRaw = frameSet->getFrame(OB_FRAME_DEPTH);
        if (depthFrameRaw == nullptr) {
            result.errorCode = OrbbecCaptureErrorCode::CaptureFailed;
            result.errorMessage = QStringLiteral("Depth frame missing in frameset");
            result.captureDurationMs = timer.elapsed();
            emit captureFinished(result);
            emit stateChanged(OrbbecGeminiRuntimeState::Ready, result.errorMessage);
            return;
        }
        const auto depthFrame = depthFrameRaw->as<ob::DepthFrame>();
        result.depthWidth = static_cast<int>(depthFrame->getWidth());
        result.depthHeight = static_cast<int>(depthFrame->getHeight());
        result.depthValueScale = depthFrame->getValueScale();

        const auto* depthData =
            reinterpret_cast<const uint16_t*>(depthFrame->getData());

        std::vector<OrbbecPointView> points;
        const bool hasPointCloud = extractPointCloud(*m_impl->pipeline, frameSet, &points);
        if (hasPointCloud) {
            result.pointCloudPointCount = static_cast<int>(points.size());
        }

        if (saveToDisk) {
            // 生成带 requestId 与时间戳的唯一文件名，写入 Orbbec 缓存子目录
            const QString timestamp = scan_tracking::common::buildCaptureTimestamp();
            buildOrbbecCapturePaths(
                m_config.captureCacheRoot,
                request.requestId,
                timestamp,
                &result.depthRawPngPath,
                &result.depthPreviewPngPath,
                &result.pointCloudPlyPath);

            OrbbecDepthFrameView depthView;
            depthView.data = depthData;
            depthView.width = result.depthWidth;
            depthView.height = result.depthHeight;
            depthView.valueScale = result.depthValueScale;

            if (!saveDepthFramePngs(
                    depthView,
                    result.depthRawPngPath,
                    result.depthPreviewPngPath,
                    &result.validDepthPixelCount)) {
                result.errorCode = OrbbecCaptureErrorCode::SaveFailed;
                result.errorMessage = QStringLiteral("Failed to save depth PNG files");
            } else if (hasPointCloud
                       && !savePointCloudPly(points, result.pointCloudPlyPath)) {
                result.errorCode = OrbbecCaptureErrorCode::SaveFailed;
                result.errorMessage = QStringLiteral("Failed to save point cloud PLY");
            }
        } else {
            // 不落盘时仅统计有效深度像素数
            int validCount = 0;
            const int pixelCount = result.depthWidth * result.depthHeight;
            for (int index = 0; index < pixelCount; ++index) {
                if (depthData[index] != 0) {
                    ++validCount;
                }
            }
            result.validDepthPixelCount = validCount;
        }

        result.captureDurationMs = timer.elapsed();
        if (result.errorCode == OrbbecCaptureErrorCode::Success) {
            emit logMessage(
                QStringLiteral("%1 Capture ok req=%2 size=%3x%4 validDepth=%5 points=%6 elapsed=%7ms")
                    .arg(logPrefix())
                    .arg(request.requestId)
                    .arg(result.depthWidth)
                    .arg(result.depthHeight)
                    .arg(result.validDepthPixelCount)
                    .arg(result.pointCloudPointCount)
                    .arg(result.captureDurationMs));
            if (saveToDisk) {
                emit logMessage(
                    QStringLiteral("%1 Saved depthRaw=%2 depthPreview=%3 pointCloud=%4")
                        .arg(logPrefix())
                        .arg(result.depthRawPngPath)
                        .arg(result.depthPreviewPngPath)
                        .arg(result.pointCloudPlyPath));
            }
        }

        emit captureFinished(result);
        emit stateChanged(
            OrbbecGeminiRuntimeState::Ready,
            result.errorCode == OrbbecCaptureErrorCode::Success
                ? QStringLiteral("Capture finished")
                : result.errorMessage);
    } catch (const ob::Error& error) {
        result.errorCode = OrbbecCaptureErrorCode::CaptureFailed;
        result.errorMessage = QString::fromStdString(error.what());
        result.captureDurationMs = timer.elapsed();
        emit logMessage(QStringLiteral("%1 Capture failed: %2").arg(logPrefix(), result.errorMessage));
        emit captureFinished(result);
        emit stateChanged(OrbbecGeminiRuntimeState::Ready, result.errorMessage);
    } catch (const std::exception& error) {
        result.errorCode = OrbbecCaptureErrorCode::CaptureFailed;
        result.errorMessage = QString::fromUtf8(error.what());
        result.captureDurationMs = timer.elapsed();
        emit logMessage(QStringLiteral("%1 Capture failed: %2").arg(logPrefix(), result.errorMessage));
        emit captureFinished(result);
        emit stateChanged(OrbbecGeminiRuntimeState::Ready, result.errorMessage);
    }
}

}  // namespace orbbec_gemini
}  // namespace scan_tracking
