#include "scan_tracking/vision/hik_cxp_camera_service.h"

#include "scan_tracking/vision/hik_mvs_sdk_runtime.h"

#include <QtCore/QDateTime>
#include <QtCore/QElapsedTimer>
#include <QtCore/QMetaObject>
#include <QtCore/QMutex>
#include <QtCore/QMutexLocker>
#include <QtCore/QThread>

#include <algorithm>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>
#include <qdebug.h>
#include "MvCameraControl.h"

namespace scan_tracking {
namespace vision {

namespace {

QString trimSdkString(const unsigned char* raw, std::size_t maxLength)
{
    if (raw == nullptr || maxLength == 0) {
        return {};
    }
    std::size_t length = 0;
    while (length < maxLength && raw[length] != '\0') {
        ++length;
    }
    return QString::fromLocal8Bit(reinterpret_cast<const char*>(raw), static_cast<int>(length))
        .trimmed();
}

bool containsKey(const QString& text, const QString& key)
{
    return !text.isEmpty() && !key.isEmpty() && text.contains(key, Qt::CaseInsensitive);
}

void extractCxpDeviceInfo(
    const MV_CC_DEVICE_INFO* deviceInfo,
    QString* serialNumber,
    QString* modelName,
    QString* userDefinedName,
    QString* deviceId,
    QString* interfaceId)
{
    if (deviceInfo == nullptr || deviceInfo->nTLayerType != MV_GENTL_CXP_DEVICE) {
        return;
    }
    const MV_CXP_DEVICE_INFO& info = deviceInfo->SpecialInfo.stCXPInfo;
    if (serialNumber != nullptr) {
        *serialNumber = trimSdkString(info.chSerialNumber, sizeof(info.chSerialNumber));
    }
    if (modelName != nullptr) {
        *modelName = trimSdkString(info.chModelName, sizeof(info.chModelName));
    }
    if (userDefinedName != nullptr) {
        *userDefinedName = trimSdkString(info.chUserDefinedName, sizeof(info.chUserDefinedName));
    }
    if (deviceId != nullptr) {
        *deviceId = trimSdkString(info.chDeviceID, sizeof(info.chDeviceID));
    }
    if (interfaceId != nullptr) {
        *interfaceId = trimSdkString(info.chInterfaceID, sizeof(info.chInterfaceID));
    }
}

bool deviceMatchesKey(
    const MV_CC_DEVICE_INFO* deviceInfo,
    const QString& preferredCameraKey)
{
    if (deviceInfo == nullptr || preferredCameraKey.isEmpty()) {
        return preferredCameraKey.isEmpty();
    }

    QString serialNumber;
    QString modelName;
    QString userDefinedName;
    QString deviceId;
    extractCxpDeviceInfo(deviceInfo, &serialNumber, &modelName, &userDefinedName, &deviceId, nullptr);

    return preferredCameraKey.compare(serialNumber, Qt::CaseInsensitive) == 0 ||
           preferredCameraKey.compare(userDefinedName, Qt::CaseInsensitive) == 0 ||
           preferredCameraKey.compare(modelName, Qt::CaseInsensitive) == 0 ||
           preferredCameraKey.compare(deviceId, Qt::CaseInsensitive) == 0 ||
           containsKey(serialNumber, preferredCameraKey) ||
           containsKey(userDefinedName, preferredCameraKey) ||
           containsKey(modelName, preferredCameraKey) ||
           containsKey(deviceId, preferredCameraKey);
}

void logDiscoveredCxpDevices(const MV_CC_DEVICE_INFO_LIST& deviceList)
{
    qInfo() << QStringLiteral("[CXP] 枚举到 %1 台 CoaXPress 设备").arg(deviceList.nDeviceNum);
    for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
        const MV_CC_DEVICE_INFO* deviceInfo = deviceList.pDeviceInfo[i];
        if (deviceInfo == nullptr) {
            continue;
        }
        QString serialNumber;
        QString modelName;
        QString userDefinedName;
        QString deviceId;
        QString interfaceId;
        extractCxpDeviceInfo(
            deviceInfo, &serialNumber, &modelName, &userDefinedName, &deviceId, &interfaceId);
        qInfo().noquote()
            << QStringLiteral("[CXP]  [%1] model=%2 sn=%3 user=%4 deviceId=%5 iface=%6")
                   .arg(i)
                   .arg(modelName, serialNumber, userDefinedName, deviceId, interfaceId);
    }
}

/// 与工位一对齐：A/B 并行枚举不安全，全局串行化 SDK 调用。
QMutex g_cxpSdkMutex;

}  // namespace

class HikCxpCameraService::Impl {
public:
    QMutex mutex;
    void* handle = nullptr;
    MV_CC_DEVICE_INFO deviceInfo{};
    QString serialNumber;
    QString modelName;
    QString userDefinedName;
    QString deviceId;
    QString interfaceId;
    bool sdkReady = false;
    bool connected = false;
    bool grabbing = false;
};

void HikCxpCameraService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::VisionErrorCode>("scan_tracking::vision::VisionErrorCode");
    qRegisterMetaType<scan_tracking::vision::HikMonoFrame>("scan_tracking::vision::HikMonoFrame");
    qRegisterMetaType<scan_tracking::vision::HikPoseCaptureResult>(
        "scan_tracking::vision::HikPoseCaptureResult");
    registered = true;
}

HikCxpCameraService::HikCxpCameraService(const QString& roleName, QObject* parent)
    : QObject(parent)
    , m_roleName(roleName)
    , m_acceptAsyncResults(std::make_shared<std::atomic_bool>(false))
    , m_impl(new Impl())
{
    registerMetaTypes();
}

HikCxpCameraService::~HikCxpCameraService()
{
    stop();
    delete m_impl;
    m_impl = nullptr;
}

void HikCxpCameraService::start(
    const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
    int defaultCaptureTimeoutMs,
    float exposureTimeUs,
    float gain)
{
    m_endpointConfig = endpointConfig;
    m_defaultCaptureTimeoutMs = defaultCaptureTimeoutMs > 0 ? defaultCaptureTimeoutMs : 5000;
    m_exposureTimeUs = exposureTimeUs > 0.0f ? exposureTimeUs : 50000.0f;
    m_gain = gain;

    QString sdkError;
    if (!acquireHikMvsSdk(&sdkError)) {
        emit fatalError(VisionErrorCode::SdkInitFailed, sdkError);
        return;
    }
    m_impl->sdkReady = true;

    m_acceptAsyncResults->store(true, std::memory_order_release);
    m_started = true;
    emit stateChanged(
        m_roleName,
        QStringLiteral("ready"),
        QStringLiteral("CXP 相机服务已启动，后台正在尝试连接设备。"));
    startAsyncConnect();
}

void HikCxpCameraService::joinWorkerThreads()
{
    {
        std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
        m_captureWorkerStopping = true;
    }
    m_captureQueueCv.notify_all();

    std::thread connectThread;
    std::thread captureThread;
    {
        std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
        connectThread = std::move(m_connectThread);
        captureThread = std::move(m_captureThread);
    }
    if (connectThread.joinable()) {
        connectThread.join();
    }
    if (captureThread.joinable()) {
        captureThread.join();
    }

    std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
    m_captureQueue.clear();
    m_captureWorkerRunning = false;
    m_captureWorkerStopping = false;
}

void HikCxpCameraService::ensureCaptureWorkerRunning()
{
    if (m_captureWorkerRunning) {
        return;
    }
    m_captureThread = std::thread([this]() { captureWorkerLoop(); });
    m_captureWorkerRunning = true;
}

quint64 HikCxpCameraService::enqueueCaptureJob(CaptureJob job)
{
    const quint64 requestId = job.seedResult.requestId;
    const auto acceptResults = m_acceptAsyncResults;

    {
        std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
        if (m_captureWorkerStopping ||
            !m_started ||
            acceptResults == nullptr ||
            !acceptResults->load(std::memory_order_acquire)) {
            return 0;
        }
        m_captureQueue.push_back(std::move(job));
        m_captureInFlight.store(true, std::memory_order_release);
        ensureCaptureWorkerRunning();
    }
    m_captureQueueCv.notify_one();
    return requestId;
}

void HikCxpCameraService::captureWorkerLoop()
{
    const auto acceptResults = m_acceptAsyncResults;
    HikCxpCameraService* const receiver = this;

    for (;;) {
        CaptureJob job;
        {
            std::unique_lock<std::mutex> lock(m_workerThreadsMutex);
            m_captureQueueCv.wait(lock, [this]() {
                return m_captureWorkerStopping || !m_captureQueue.empty();
            });
            if (m_captureWorkerStopping && m_captureQueue.empty()) {
                break;
            }
            job = std::move(m_captureQueue.front());
            m_captureQueue.pop_front();
        }

        QElapsedTimer timer;
        timer.start();
        QString errorMessage;

        if (!receiver->ensureConnected(job.preferredCameraKey, &errorMessage)) {
            job.seedResult.errorCode = VisionErrorCode::DeviceOpenFailed;
            job.seedResult.errorMessage = errorMessage;
            job.seedResult.elapsedMs = timer.elapsed();
        } else {
            HikMonoFrame capturedFrame;
            if (!receiver->captureMonoFrame(
                    job.effectiveTimeoutMs,
                    job.seedResult.cameraKey,
                    &errorMessage,
                    &capturedFrame)) {
                job.seedResult.errorCode = VisionErrorCode::CaptureRejected;
                job.seedResult.errorMessage = errorMessage.isEmpty()
                    ? QStringLiteral("CXP Mono 采图失败。")
                    : errorMessage;
                job.seedResult.elapsedMs = timer.elapsed();
            } else {
                job.seedResult.errorCode = VisionErrorCode::Success;
                job.seedResult.errorMessage = QStringLiteral("CXP Mono 采图完成。");
                job.seedResult.frame = std::move(capturedFrame);
                job.seedResult.frame.sourceCameraKey = job.seedResult.cameraKey;
                job.seedResult.elapsedMs = timer.elapsed();
            }
        }

        {
            std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
            if (m_captureQueue.empty()) {
                m_captureInFlight.store(false, std::memory_order_release);
            }
        }

        if (!acceptResults->load(std::memory_order_acquire)) {
            continue;
        }

        QMetaObject::invokeMethod(
            receiver,
            [acceptResults, receiver, seedResult = std::move(job.seedResult)]() mutable {
                if (!acceptResults->load(std::memory_order_acquire)) {
                    return;
                }
                const bool ok = seedResult.success();
                emit receiver->monoCaptureFinished(seedResult);
                emit receiver->poseCaptureFinished(std::move(seedResult));
                emit receiver->stateChanged(
                    receiver->m_roleName,
                    QStringLiteral("ready"),
                    ok ? QStringLiteral("CXP 采图完成。")
                       : QStringLiteral("CXP 采图失败。"));
            },
            Qt::QueuedConnection);
    }

    std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
    m_captureWorkerRunning = false;
    m_captureInFlight.store(false, std::memory_order_release);
}

void HikCxpCameraService::stop()
{
    m_started = false;
    m_acceptAsyncResults->store(false, std::memory_order_release);

    if (m_impl != nullptr && m_impl->handle != nullptr) {
        MV_CC_StopGrabbing(m_impl->handle);
    }

    joinWorkerThreads();
    m_connectInFlight.store(false, std::memory_order_release);
    m_captureInFlight.store(false, std::memory_order_release);

    closeDevice();

    if (m_impl != nullptr && m_impl->sdkReady) {
        releaseHikMvsSdk();
        m_impl->sdkReady = false;
    }

    emit stateChanged(m_roleName, QStringLiteral("stopped"), QStringLiteral("CXP 相机服务已停止。"));
}

bool HikCxpCameraService::isStarted() const
{
    return m_started;
}

bool HikCxpCameraService::isConnected() const
{
    if (m_impl == nullptr) {
        return false;
    }
    QMutexLocker locker(&m_impl->mutex);
    return m_impl->connected;
}

QString HikCxpCameraService::roleName() const
{
    return m_roleName;
}

QString HikCxpCameraService::resolveCameraKey(const QString& preferredCameraKey) const
{
    if (!preferredCameraKey.trimmed().isEmpty()) {
        return preferredCameraKey.trimmed();
    }
    if (!m_endpointConfig.cameraKey.trimmed().isEmpty()) {
        return m_endpointConfig.cameraKey.trimmed();
    }
    if (!m_endpointConfig.serialNumber.trimmed().isEmpty()) {
        return m_endpointConfig.serialNumber.trimmed();
    }
    return m_endpointConfig.logicalName.trimmed();
}

bool HikCxpCameraService::captureMonoFrame(
    int timeoutMs,
    const QString& cameraKey,
    QString* errorMessage,
    HikMonoFrame* outFrame)
{
    void* handle = nullptr;
    {
        QMutexLocker locker(&m_impl->mutex);
        if (m_impl->handle == nullptr || !m_impl->connected) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("CXP 相机尚未连接，无法采图：%1").arg(cameraKey);
            }
            return false;
        }
        if (!m_started) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("CXP 相机服务正在停止，取消采图");
            }
            return false;
        }
        handle = m_impl->handle;
    }

    const unsigned int waitMs = static_cast<unsigned int>(timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs);
    const unsigned int actualWaitMs = waitMs < 5000 ? 5000 : waitMs;

    // CXP 相机连接后处于连续自由采集。仅调用 ClearImageBuffer 只能清 SDK
    // 输出队列，不能清掉采集卡/链路上的在途帧；路径切换后的首点可能因此取到
    // 上一段的旧图。每次请求都重置一次采集状态，保证本次等待从新采集周期开始。
    const int stopResult = MV_CC_StopGrabbing(handle);
    if (stopResult != MV_OK && stopResult != MV_E_CALLORDER) {
        qWarning() << QStringLiteral("[%1] CXP 采图前 StopGrabbing 失败 0x%2")
                          .arg(m_roleName)
                          .arg(static_cast<quint32>(stopResult), 8, 16, QLatin1Char('0'));
    }
    {
        QMutexLocker locker(&m_impl->mutex);
        m_impl->grabbing = false;
    }

    QElapsedTimer grabTimer;
    grabTimer.start();
    const int clearResult = MV_CC_ClearImageBuffer(handle);
    qInfo() << QStringLiteral("[%1] CXP 采图前 ClearImageBuffer 结果=0x%2（丢弃队列旧帧）")
                   .arg(m_roleName)
                   .arg(static_cast<quint32>(clearResult), 8, 16, QLatin1Char('0'));

    const int startResult = MV_CC_StartGrabbing(handle);
    if (startResult != MV_OK && startResult != MV_E_CALLORDER) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("MV_CC_StartGrabbing 失败，错误码=0x%1")
                                .arg(static_cast<quint32>(startResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }
    {
        QMutexLocker locker(&m_impl->mutex);
        m_impl->grabbing = true;
    }

    const int strategyResult =
        MV_CC_SetGrabStrategy(handle, MV_GrabStrategy_LatestImagesOnly);
    qInfo() << QStringLiteral("[%1] CXP 采图前设置 GrabStrategy=LatestImagesOnly 结果=0x%2")
                   .arg(m_roleName)
                   .arg(static_cast<quint32>(strategyResult), 8, 16, QLatin1Char('0'));

    MV_FRAME_OUT frameOut{};
    const int getBufferResult = MV_CC_GetImageBuffer(handle, &frameOut, actualWaitMs);

    if (!m_started) {
        if (getBufferResult == MV_OK && frameOut.pBufAddr != nullptr) {
            MV_CC_FreeImageBuffer(handle, &frameOut);
        }
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 相机服务正在停止，采图被中断");
        }
        return false;
    }

    if (getBufferResult == MV_OK && frameOut.pBufAddr != nullptr) {
        const int width = static_cast<int>(frameOut.stFrameInfo.nWidth);
        const int height = static_cast<int>(frameOut.stFrameInfo.nHeight);
        const int frameLen = static_cast<int>(frameOut.stFrameInfo.nFrameLen);

        if (width > 0 && height > 0 && frameLen > 0) {
            auto pixels = std::make_shared<std::vector<std::uint8_t>>();
            pixels->assign(
                static_cast<unsigned char*>(frameOut.pBufAddr),
                static_cast<unsigned char*>(frameOut.pBufAddr) + frameLen);

            HikMonoFrame frame;
            frame.pixels = std::move(pixels);
            frame.width = width;
            frame.height = height;
            frame.stride = width;
            frame.frameId = frameOut.stFrameInfo.nFrameNum;
            frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
            frame.sourceCameraKey = cameraKey;
            frame.pixelFormat = QStringLiteral("Mono8");

            MV_CC_FreeImageBuffer(handle, &frameOut);

            if (frame.isValid()) {
                if (outFrame != nullptr) {
                    *outFrame = frame;
                }
                if (errorMessage != nullptr) {
                    errorMessage->clear();
                }
                qInfo() << QStringLiteral(
                               "[%1] CXP 采图成功 %2x%3 len=%4 frameId=%5 elapsedMs=%6 "
                               "(清缓冲后取最新帧)")
                               .arg(m_roleName)
                               .arg(width)
                               .arg(height)
                               .arg(frameLen)
                               .arg(frame.frameId)
                               .arg(grabTimer.elapsed());
                return true;
            }
        }
        MV_CC_FreeImageBuffer(handle, &frameOut);
    }

    thread_local std::vector<unsigned char> fallbackBuffer;
    if (fallbackBuffer.size() < 48U * 1024U * 1024U) {
        fallbackBuffer.resize(48U * 1024U * 1024U);
    }
    MV_FRAME_OUT_INFO_EX frameInfo{};
    const int grabResult = MV_CC_GetOneFrameTimeout(
        handle,
        fallbackBuffer.data(),
        static_cast<unsigned int>(fallbackBuffer.size()),
        &frameInfo,
        actualWaitMs);

    if (!m_started) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 相机服务正在停止，采图被中断");
        }
        return false;
    }

    if (grabResult != MV_OK) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 采图失败(GetImageBuffer=0x%1, GetOneFrame=0x%2)")
                                .arg(static_cast<quint32>(getBufferResult), 8, 16, QLatin1Char('0'))
                                .arg(static_cast<quint32>(grabResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    const int width = static_cast<int>(frameInfo.nWidth);
    const int height = static_cast<int>(frameInfo.nHeight);
    const int frameLen = static_cast<int>(frameInfo.nFrameLen);
    if (width <= 0 || height <= 0 || frameLen <= 0 || frameLen > static_cast<int>(fallbackBuffer.size())) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 图像帧信息无效 width=%1 height=%2 len=%3")
                                .arg(width)
                                .arg(height)
                                .arg(frameLen);
        }
        return false;
    }

    auto pixels = std::make_shared<std::vector<std::uint8_t>>();
    pixels->assign(fallbackBuffer.begin(), fallbackBuffer.begin() + frameLen);

    HikMonoFrame frame;
    frame.pixels = std::move(pixels);
    frame.width = width;
    frame.height = height;
    frame.stride = width;
    frame.frameId = frameInfo.nFrameNum;
    frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
    frame.sourceCameraKey = cameraKey;
    frame.pixelFormat = QStringLiteral("Mono8");

    if (!frame.isValid()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 采图成功但帧无效");
        }
        return false;
    }

    if (outFrame != nullptr) {
        *outFrame = frame;
    }
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    qInfo() << QStringLiteral(
                   "[%1] CXP 采图成功(GetOneFrame) %2x%3 frameId=%4 elapsedMs=%5 "
                   "(清缓冲后取最新帧)")
                   .arg(m_roleName)
                   .arg(width)
                   .arg(height)
                   .arg(frame.frameId)
                   .arg(grabTimer.elapsed());
    return true;
}

quint64 HikCxpCameraService::requestMonoCapture(const QString& preferredCameraKey, int timeoutMs)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("CXP 相机服务尚未启动。"));
        return 0;
    }

    CaptureJob job;
    job.seedResult.requestId = m_nextRequestId++;
    job.seedResult.cameraKey = resolveCameraKey(preferredCameraKey);
    job.seedResult.logicalName = m_endpointConfig.logicalName;
    job.preferredCameraKey = preferredCameraKey;
    job.effectiveTimeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;

    return enqueueCaptureJob(std::move(job));
}

bool HikCxpCameraService::ensureConnected(const QString& preferredCameraKey, QString* errorMessage)
{
    return openMatchedDevice(resolveCameraKey(preferredCameraKey), errorMessage);
}

bool HikCxpCameraService::openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage)
{
    // 与工位一相同：EnumDevices(MV_GENTL_CXP_DEVICE) → CreateHandle → OpenDevice。
    // 依赖 GENICAM_GENTL64_PATH 指向含 MvFGProducerCXP.cti 的目录（ensureHikGenTlEnvironment）。
    // 锁序：本实例 lifecycle → 全局 SDK → impl，避免连接/采图并行开设备。
    std::lock_guard<std::mutex> lifecycleLock(m_deviceLifecycleMutex);
    QMutexLocker sdkLocker(&g_cxpSdkMutex);

    if (m_impl == nullptr || !m_impl->sdkReady) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("MVS SDK 尚未初始化。");
        }
        return false;
    }

    {
        QMutexLocker locker(&m_impl->mutex);
        if (m_impl->connected && m_impl->handle != nullptr) {
            if (errorMessage != nullptr) {
                errorMessage->clear();
            }
            return true;
        }
    }

    closeDeviceUnlocked();

    MV_CC_DEVICE_INFO_LIST deviceList{};
    const int enumResult = MV_CC_EnumDevices(MV_GENTL_CXP_DEVICE, &deviceList);
    if (enumResult != MV_OK) {
        if (errorMessage != nullptr) {
            const quint32 code = static_cast<quint32>(enumResult);
            QString hint;
            if (code == 0x800000FFu) {
                const QByteArray gentl = qgetenv("GENICAM_GENTL64_PATH");
                hint = QStringLiteral(
                    "（MV_E_UNKNOW：GenTL/CXP 枚举失败。GENICAM_GENTL64_PATH=%1；"
                    "请确认存在 MvFGProducerCXP.cti，并与工位一一样用 start.bat 启动）")
                           .arg(gentl.isEmpty() ? QStringLiteral("<空>")
                                                : QString::fromLocal8Bit(gentl));
            }
            *errorMessage = QStringLiteral("枚举 CXP 相机失败，错误码=0x%1%2")
                                .arg(code, 8, 16, QLatin1Char('0'))
                                .arg(hint);
        }
        return false;
    }

    logDiscoveredCxpDevices(deviceList);

    MV_CC_DEVICE_INFO* matchedDevice = nullptr;
    QString matchedSerial;
    QString matchedModel;
    QString matchedUser;
    QString matchedDeviceId;
    QString matchedInterfaceId;

    for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* deviceInfo = deviceList.pDeviceInfo[i];
        if (deviceInfo == nullptr || deviceInfo->nTLayerType != MV_GENTL_CXP_DEVICE) {
            continue;
        }

        if (!preferredCameraKey.isEmpty() && !deviceMatchesKey(deviceInfo, preferredCameraKey)) {
            continue;
        }

        matchedDevice = deviceInfo;
        extractCxpDeviceInfo(
            deviceInfo,
            &matchedSerial,
            &matchedModel,
            &matchedUser,
            &matchedDeviceId,
            &matchedInterfaceId);
        break;
    }

    if (matchedDevice == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("未找到匹配的 CXP 相机：%1").arg(preferredCameraKey);
        }
        return false;
    }

    void* handle = nullptr;
    const int createResult = MV_CC_CreateHandle(&handle, matchedDevice);
    if (createResult != MV_OK || handle == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("创建 CXP 相机句柄失败，错误码=0x%1")
                                .arg(static_cast<quint32>(createResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    const int openResult = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
    if (openResult != MV_OK) {
        MV_CC_DestroyHandle(handle);
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("打开 CXP 相机失败，错误码=0x%1")
                                .arg(static_cast<quint32>(openResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    int ret = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 TriggerMode 失败 0x%2")
                          .arg(m_roleName)
                          .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
    }

    ret = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080001);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 Mono8 失败（可能已是其他格式）0x%2")
                          .arg(m_roleName)
                          .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
    }

    ret = MV_CC_SetFloatValue(handle, "ExposureTime", m_exposureTimeUs);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 ExposureTime 失败").arg(m_roleName);
    }

    ret = MV_CC_SetFloatValue(handle, "Gain", m_gain);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 Gain 失败").arg(m_roleName);
    }

    ret = MV_CC_StartGrabbing(handle);
    if (ret != MV_OK && ret != MV_E_CALLORDER) {
        qWarning() << QStringLiteral("[%1] StartGrabbing 失败 0x%2")
                          .arg(m_roleName)
                          .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
    } else {
        // 连接后即自由跑；每次采图前仍会 ClearImageBuffer + LatestImagesOnly。
        const int strategyAtOpen =
            MV_CC_SetGrabStrategy(handle, MV_GrabStrategy_LatestImagesOnly);
        qInfo() << QStringLiteral(
                       "[%1] CXP 已 StartGrabbing(TriggerMode=Off 连续采)，"
                       "GrabStrategy=LatestImagesOnly 结果=0x%2")
                       .arg(m_roleName)
                       .arg(static_cast<quint32>(strategyAtOpen), 8, 16, QLatin1Char('0'));
        QThread::msleep(100);
    }

    {
        QMutexLocker locker(&m_impl->mutex);
        m_impl->handle = handle;
        m_impl->deviceInfo = *matchedDevice;
        m_impl->serialNumber = matchedSerial;
        m_impl->modelName = matchedModel;
        m_impl->userDefinedName = matchedUser;
        m_impl->deviceId = matchedDeviceId;
        m_impl->interfaceId = matchedInterfaceId;
        m_impl->connected = true;
        m_impl->grabbing = (ret == MV_OK || ret == MV_E_CALLORDER);
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    qInfo().noquote() << QStringLiteral("[%1] CXP 已连接 sn=%2 model=%3 iface=%4")
                             .arg(m_roleName, matchedSerial, matchedModel, matchedInterfaceId);
    return true;
}

void HikCxpCameraService::closeDeviceUnlocked()
{
    if (m_impl == nullptr) {
        return;
    }
    QMutexLocker locker(&m_impl->mutex);
    if (m_impl->handle != nullptr) {
        MV_CC_StopGrabbing(m_impl->handle);
        MV_CC_CloseDevice(m_impl->handle);
        MV_CC_DestroyHandle(m_impl->handle);
        m_impl->handle = nullptr;
    }
    m_impl->connected = false;
    m_impl->grabbing = false;
}

void HikCxpCameraService::closeDevice()
{
    std::lock_guard<std::mutex> lifecycleLock(m_deviceLifecycleMutex);
    QMutexLocker sdkLocker(&g_cxpSdkMutex);
    closeDeviceUnlocked();
}

void HikCxpCameraService::startAsyncConnect()
{
    if (!m_started || m_connectInFlight.exchange(true)) {
        return;
    }

    const auto acceptResults = m_acceptAsyncResults;
    HikCxpCameraService* const receiver = this;

    std::thread finishedThread;
    {
        std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
        if (!m_started ||
            acceptResults == nullptr ||
            !acceptResults->load(std::memory_order_acquire)) {
            m_connectInFlight.store(false, std::memory_order_release);
            return;
        }
        if (m_connectThread.joinable()) {
            finishedThread = std::move(m_connectThread);
        }
        m_connectThread = std::thread([acceptResults, receiver]() {
            const QString cameraKey = receiver->resolveCameraKey({});
            QString errorMessage;
            const bool ok = receiver->openMatchedDevice(cameraKey, &errorMessage);

            receiver->m_connectInFlight.store(false, std::memory_order_release);

            if (!acceptResults->load(std::memory_order_acquire)) {
                return;
            }

            QMetaObject::invokeMethod(
                receiver,
                [acceptResults, receiver, ok, cameraKey, errorMessage]() {
                    if (!acceptResults->load(std::memory_order_acquire) || !receiver->m_started) {
                        return;
                    }
                    if (ok) {
                        emit receiver->stateChanged(
                            receiver->m_roleName,
                            QStringLiteral("connected"),
                            QStringLiteral("CXP 相机已连接：%1 (%2)")
                                .arg(receiver->m_impl->serialNumber, receiver->m_impl->modelName));
                    } else {
                        emit receiver->stateChanged(
                            receiver->m_roleName,
                            QStringLiteral("ready"),
                            QStringLiteral("CXP 服务已启动，尚未连接：%1").arg(errorMessage));
                        emit receiver->fatalError(
                            VisionErrorCode::DeviceNotFound,
                            QStringLiteral("后台连接 CXP 相机失败：%1").arg(cameraKey));
                    }
                },
                Qt::QueuedConnection);
        });
    }

    if (finishedThread.joinable()) {
        finishedThread.join();
    }
}

}  // namespace vision
}  // namespace scan_tracking
