#include "scan_tracking/vision/hik_camera_service.h"

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
#include<qdebug.h>
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
    return QString::fromLocal8Bit(reinterpret_cast<const char*>(raw), static_cast<int>(length)).trimmed();
}

QString ipToString(unsigned int ip)
{
    return QStringLiteral("%1.%2.%3.%4")
        .arg((ip >> 24) & 0xFFu)
        .arg((ip >> 16) & 0xFFu)
        .arg((ip >> 8) & 0xFFu)
        .arg(ip & 0xFFu);
}

bool containsKey(const QString& text, const QString& key)
{
    return !text.isEmpty() && !key.isEmpty() && text.contains(key, Qt::CaseInsensitive);
}

}  // namespace

class HikCameraService::Impl {
public:
    QMutex mutex;
    void* handle = nullptr;
    MV_CC_DEVICE_INFO deviceInfo{};
    QString modelName;
    QString serialNumber;
    QString ipAddress;
    QString userDefinedName;
    QString lastPixelFormat;
    int lastFrameWidth = 0;
    int lastFrameHeight = 0;
    quint64 lastFrameId = 0;
    bool sdkReady = false;
    bool connected = false;
};

void HikCameraService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::VisionErrorCode>("scan_tracking::vision::VisionErrorCode");
    qRegisterMetaType<scan_tracking::vision::PoseMatrix4x4>("scan_tracking::vision::PoseMatrix4x4");
    qRegisterMetaType<scan_tracking::vision::HikMonoFrame>("scan_tracking::vision::HikMonoFrame");
    qRegisterMetaType<scan_tracking::vision::HikPoseCaptureRequest>("scan_tracking::vision::HikPoseCaptureRequest");
    qRegisterMetaType<scan_tracking::vision::HikPoseCaptureResult>("scan_tracking::vision::HikPoseCaptureResult");
    registered = true;
}

HikCameraService::HikCameraService(const QString& roleName, QObject* parent)
    : QObject(parent)
    , m_roleName(roleName)
    , m_acceptAsyncResults(std::make_shared<std::atomic_bool>(false))
    , m_impl(new Impl())
{
    registerMetaTypes();
}

HikCameraService::~HikCameraService()
{
    stop();
    delete m_impl;
    m_impl = nullptr;
}

void HikCameraService::start(
    const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
    int defaultCaptureTimeoutMs,
    float exposureTimeUs,
    float gain)
{
    m_endpointConfig = endpointConfig;
    m_defaultCaptureTimeoutMs = defaultCaptureTimeoutMs > 0 ? defaultCaptureTimeoutMs : 1000;
    m_exposureTimeUs = exposureTimeUs > 0.0f ? exposureTimeUs : 50000.0f;
    m_gain = gain;

    if (!m_impl->sdkReady) {
        QString sdkError;
        if (!acquireHikMvsSdk(&sdkError)) {
            emit fatalError(VisionErrorCode::SdkInitFailed, sdkError);
            return;
        }
        m_impl->sdkReady = true;
    }

    m_acceptAsyncResults->store(true, std::memory_order_release);
    m_started = true;
    emit stateChanged(m_roleName, QStringLiteral("ready"), QStringLiteral("海康相机服务已启动，后台正在尝试连接设备。"));
    startAsyncConnect();
}

void HikCameraService::joinWorkerThreads()
{
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
}

void HikCameraService::stop()
{
    m_started = false;
    m_acceptAsyncResults->store(false, std::memory_order_release);

    // 先停采集以打断 GetImageBuffer/GetOneFrameTimeout，再 join（勿在主线程 sleep 等 Queued 回调）。
    if (m_impl != nullptr && m_impl->handle != nullptr) {
        const int stopResult = MV_CC_StopGrabbing(m_impl->handle);
        if (stopResult != MV_OK) {
            qWarning() << QStringLiteral("StopGrabbing 失败，错误码=0x") << QString::number(stopResult, 16);
        }
    }

    joinWorkerThreads();
    m_connectInFlight.store(false, std::memory_order_release);
    m_captureInFlight.store(false, std::memory_order_release);

    closeDevice();

    if (m_impl != nullptr && m_impl->sdkReady) {
        releaseHikMvsSdk();
        m_impl->sdkReady = false;
    }

    emit stateChanged(m_roleName, QStringLiteral("stopped"), QStringLiteral("海康相机服务已停止。"));
}

bool HikCameraService::isStarted() const
{
    return m_started;
}

bool HikCameraService::isConnected() const
{
    if (m_impl == nullptr) {
        return false;
    }
    QMutexLocker locker(&m_impl->mutex);
    return m_impl->connected;
}

QString HikCameraService::roleName() const
{
    return m_roleName;
}

const scan_tracking::common::VisionCameraEndpointConfig& HikCameraService::endpointConfig() const
{
    return m_endpointConfig;
}

QString HikCameraService::resolveCameraKey(const QString& preferredCameraKey) const
{
    if (!preferredCameraKey.trimmed().isEmpty()) return preferredCameraKey.trimmed();
    if (!m_endpointConfig.cameraKey.trimmed().isEmpty()) return m_endpointConfig.cameraKey.trimmed();
    if (!m_endpointConfig.ipAddress.trimmed().isEmpty()) return m_endpointConfig.ipAddress.trimmed();
    if (!m_endpointConfig.serialNumber.trimmed().isEmpty()) return m_endpointConfig.serialNumber.trimmed();
    return m_endpointConfig.logicalName.trimmed();
}

bool HikCameraService::captureMonoFrame(int timeoutMs, const QString& cameraKey, QString* errorMessage, HikMonoFrame* outFrame)
{
    // 先获取 handle 的副本，避免在长时间等待期间持有锁
    void* handle = nullptr;
    {
        QMutexLocker locker(&m_impl->mutex);
        if (m_impl->handle == nullptr || !m_impl->connected) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("海康相机尚未连接，无法抓取黑白图：%1").arg(cameraKey);
            }
            return false;
        }
        
        // 检查是否正在停止
        if (!m_started) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("相机服务正在停止，取消采图");
            }
            qInfo() << "[采图] 相机服务正在停止，取消采图";
            return false;
        }
        
        handle = m_impl->handle;
    }
    // 锁已释放，可以安全地进行长时间等待

    // 增加超时时间到 5 秒
    const unsigned int waitMs = static_cast<unsigned int>(timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs);
    const unsigned int actualWaitMs = waitMs < 5000 ? 5000 : waitMs;
    
    qInfo() << "[采图] 开始采图，超时=" << actualWaitMs << "ms";
    
    // 尝试使用 MV_CC_GetImageBuffer 代替 GetOneFrameTimeout
    // 这个 API 更适合连续采集模式
    
    // 相机已经在连接时启动了采集
    const int startResult = MV_CC_StartGrabbing(handle);
    if (startResult != MV_OK && startResult != MV_E_CALLORDER) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("MV_CC_StartGrabbing 失败，错误码=0x%1")
                .arg(static_cast<quint32>(startResult), 8, 16, QLatin1Char('0'));
        }
        qWarning() << QStringLiteral("[采图] 启动采集(StartGrabbing)失败，错误码=0x") << QString::number(startResult, 16);
        return false;
    }
    if (startResult == MV_E_CALLORDER) {
        qInfo() << "[采图] 相机已在采集中";
    } else {
        qInfo() << QStringLiteral("[采图] 启动采集(StartGrabbing)成功");
    }

    qInfo() << "[采图] 等待图像数据...";
    qInfo().noquote()
        << "[ScanSync]"
        << (m_roleName.contains(QStringLiteral("_b")) ? QStringLiteral("hik_b")
                                                       : QStringLiteral("hik_a"))
        << QDateTime::currentMSecsSinceEpoch();

    // 尝试使用 GetImageBuffer（推荐用于连续采集）
    // 注意：这里不持有锁，所以 handle 可能在等待期间被销毁
    // 但 StopGrabbing 应该会中断这个调用
    MV_FRAME_OUT pFrameInfo = {0};
    int getBufferResult = MV_CC_GetImageBuffer(handle, &pFrameInfo, actualWaitMs);
    
    // 检查是否在等待期间被停止
    if (!m_started) {
        if (getBufferResult == MV_OK && pFrameInfo.pBufAddr != nullptr) {
            MV_CC_FreeImageBuffer(handle, &pFrameInfo);
        }
        if (errorMessage) {
            *errorMessage = QStringLiteral("相机服务正在停止，采图被中断");
        }
        qInfo() << "[采图] 相机服务正在停止，采图被中断";
        return false;
    }
    
    if (getBufferResult == MV_OK && pFrameInfo.pBufAddr != nullptr) {
        qInfo() << QStringLiteral("[采图] 取图缓冲(GetImageBuffer)成功");
        
        const int width = static_cast<int>(pFrameInfo.stFrameInfo.nWidth);
        const int height = static_cast<int>(pFrameInfo.stFrameInfo.nHeight);
        const int frameLen = static_cast<int>(pFrameInfo.stFrameInfo.nFrameLen);
        
        qInfo() << QStringLiteral("[采图] 图像信息：宽=") << width << QStringLiteral(" 高=") << height << QStringLiteral(" 长度=") << frameLen;
        
        if (width > 0 && height > 0 && frameLen > 0 && pFrameInfo.pBufAddr != nullptr) {
            auto pixels = std::make_shared<std::vector<std::uint8_t>>();
            pixels->assign(
                static_cast<unsigned char*>(pFrameInfo.pBufAddr),
                static_cast<unsigned char*>(pFrameInfo.pBufAddr) + frameLen);
            
            HikMonoFrame frame;
            frame.pixels = std::move(pixels);
            frame.width = width;
            frame.height = height;
            frame.stride = width;
            frame.frameId = pFrameInfo.stFrameInfo.nFrameNum;
            frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
            frame.sourceCameraKey = cameraKey;
            
            // 释放图像缓冲区
            MV_CC_FreeImageBuffer(handle, &pFrameInfo);
            
            if (frame.isValid()) {
                // 更新状态时需要加锁
                QMutexLocker locker(&m_impl->mutex);
                m_impl->lastFrameWidth = width;
                m_impl->lastFrameHeight = height;
                m_impl->lastFrameId = pFrameInfo.stFrameInfo.nFrameNum;
                m_impl->lastPixelFormat = QStringLiteral("Mono8");
                if (errorMessage) {
                    errorMessage->clear();
                }
                if (outFrame) {
                    *outFrame = frame;
                }
                qInfo() << "[采图] 采图完成，帧ID=" << pFrameInfo.stFrameInfo.nFrameNum;
                return true;
            }
        }
        
        MV_CC_FreeImageBuffer(handle, &pFrameInfo);
    }
    
    qWarning() << QStringLiteral("[采图] 取图缓冲(GetImageBuffer)失败，尝试 GetOneFrameTimeout，错误码=0x") << QString::number(getBufferResult, 16);
    
    // 如果 GetImageBuffer 失败，回退到 GetOneFrameTimeout
    std::vector<unsigned char> buffer(16 * 1024 * 1024);
    MV_FRAME_OUT_INFO_EX frameInfo;
    std::memset(&frameInfo, 0, sizeof(frameInfo));
    
    const int grabResult = MV_CC_GetOneFrameTimeout(
        handle,
        buffer.data(),
        static_cast<unsigned int>(buffer.size()),
        &frameInfo,
        actualWaitMs);
    
    // 再次检查是否在等待期间被停止
    if (!m_started) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("相机服务正在停止，采图被中断");
        }
        qInfo() << "[采图] 相机服务正在停止，采图被中断";
        return false;
    }

    if (grabResult != MV_OK) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("MV_CC_GetOneFrameTimeout 失败，错误码=0x%1，请检查：1.镜头盖是否打开 2.光照是否充足 3.相机是否正常工作")
                .arg(static_cast<quint32>(grabResult), 8, 16, QLatin1Char('0'));
        }
        qWarning() << QStringLiteral("[采图] 单帧超时(GetOneFrameTimeout)失败，错误码=0x") << QString::number(grabResult, 16);
        qWarning() << QStringLiteral("[采图] 请检查：1.镜头盖是否打开 2.光照是否充足 3.使用 MVS Viewer 测试相机");
        return false;
    }
    
    qInfo() << "[采图] 获取图像成功";

    const int width = static_cast<int>(frameInfo.nWidth);
    const int height = static_cast<int>(frameInfo.nHeight);
    const int frameLen = static_cast<int>(frameInfo.nFrameLen);
    
    qInfo() << QStringLiteral("[采图] 图像信息：宽=") << width << QStringLiteral(" 高=") << height << QStringLiteral(" 长度=") << frameLen;
    
    if (width <= 0 || height <= 0 || frameLen <= 0 || frameLen > static_cast<int>(buffer.size())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("海康图像帧信息无效。width=%1 height=%2 len=%3")
                .arg(width).arg(height).arg(frameLen);
        }
        return false;
    }

    auto pixels = std::make_shared<std::vector<std::uint8_t>>();
    pixels->assign(buffer.begin(), buffer.begin() + frameLen);

    HikMonoFrame frame;
    frame.pixels = std::move(pixels);
    frame.width = width;
    frame.height = height;
    frame.stride = width;
    frame.frameId = frameInfo.nFrameNum;
    frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
    frame.sourceCameraKey = cameraKey;

    if (!frame.isValid()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("海康采图成功但黑白帧无效。");
        }
        return false;
    }

    // 更新状态时需要加锁
    {
        QMutexLocker locker(&m_impl->mutex);
        m_impl->lastFrameWidth = width;
        m_impl->lastFrameHeight = height;
        m_impl->lastFrameId = frameInfo.nFrameNum;
        m_impl->lastPixelFormat = QStringLiteral("Mono8");
    }
    
    if (outFrame) {
        *outFrame = frame;
    }
    
    if (errorMessage) {
        errorMessage->clear();
    }
    
    qInfo() << "[采图] 采图完成，帧ID=" << frameInfo.nFrameNum;
    return true;
}

quint64 HikCameraService::requestPoseCapture(const QString& preferredCameraKey, int timeoutMs)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("海康相机服务尚未启动。"));
        return 0;
    }
    if (m_captureInFlight.exchange(true)) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("海康相机采集请求正在执行，请稍后重试。"));
        return 0;
    }

    HikPoseCaptureResult seedResult;
    seedResult.requestId = m_nextRequestId++;
    seedResult.cameraKey = resolveCameraKey(preferredCameraKey);
    seedResult.logicalName = m_endpointConfig.logicalName;
    const quint64 requestId = seedResult.requestId;
    const int effectiveTimeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;
    const auto acceptResults = m_acceptAsyncResults;
    HikCameraService* const receiver = this;

    std::thread finishedThread;
    {
        std::lock_guard<std::mutex> lock(m_workerThreadsMutex);
        if (!m_started ||
            acceptResults == nullptr ||
            !acceptResults->load(std::memory_order_acquire)) {
            m_captureInFlight.store(false, std::memory_order_release);
            return 0;
        }
        if (m_captureThread.joinable()) {
            finishedThread = std::move(m_captureThread);
        }
        // pixels 已是 shared_ptr，进线程时帧为空；采图后 move 回投，避免多余拷贝。
        m_captureThread = std::thread(
            [acceptResults, receiver, preferredCameraKey, effectiveTimeoutMs,
             seedResult = std::move(seedResult)]() mutable {
                QElapsedTimer timer;
                timer.start();
                QString errorMessage;

                if (!receiver->ensureConnected(preferredCameraKey, &errorMessage)) {
                    seedResult.errorCode = VisionErrorCode::DeviceOpenFailed;
                    seedResult.errorMessage = errorMessage;
                    seedResult.elapsedMs = timer.elapsed();
                } else {
                    HikMonoFrame capturedFrame;
                    if (!receiver->captureMonoFrame(
                            effectiveTimeoutMs,
                            seedResult.cameraKey,
                            &errorMessage,
                            &capturedFrame)) {
                        seedResult.errorCode = VisionErrorCode::CaptureRejected;
                        seedResult.errorMessage = errorMessage.isEmpty()
                            ? QStringLiteral("海康 Mono8 采图失败。")
                            : errorMessage;
                        seedResult.elapsedMs = timer.elapsed();
                    } else {
                        seedResult.errorCode = VisionErrorCode::Success;
                        seedResult.errorMessage = QStringLiteral("海康 Mono8 黑白采图完成。");
                        seedResult.frame = std::move(capturedFrame);
                        seedResult.frame.frameId = seedResult.requestId;
                        seedResult.frame.sourceCameraKey = seedResult.cameraKey;
                        seedResult.elapsedMs = timer.elapsed();
                        qInfo() << QStringLiteral("[") << receiver->m_roleName
                                << QStringLiteral("] 采图成功：帧=")
                                << seedResult.frame.width << QStringLiteral("x")
                                << seedResult.frame.height
                                << QStringLiteral(" 像素=")
                                << (seedResult.frame.pixels ? seedResult.frame.pixels->size() : 0)
                                << QStringLiteral(" bytes");
                    }
                }

                // 先释放 busy，再回投；过期回调靠 accept 闸门丢弃。
                receiver->m_captureInFlight.store(false, std::memory_order_release);

                if (!acceptResults->load(std::memory_order_acquire)) {
                    return;
                }

                QMetaObject::invokeMethod(
                    receiver,
                    [acceptResults, receiver, seedResult = std::move(seedResult)]() mutable {
                        if (!acceptResults->load(std::memory_order_acquire)) {
                            return;
                        }
                        const bool ok = seedResult.success();
                        emit receiver->poseCaptureFinished(std::move(seedResult));
                        emit receiver->stateChanged(
                            receiver->m_roleName,
                            QStringLiteral("ready"),
                            ok ? QStringLiteral("海康相机完成 Mono8 黑白采图，等待下一次请求。")
                               : QStringLiteral("海康相机黑白采图失败，请检查相机状态。"));
                    },
                    Qt::QueuedConnection);
            });
    }

    if (finishedThread.joinable()) {
        finishedThread.join();
    }
    return requestId;
}

bool HikCameraService::ensureConnected(const QString& preferredCameraKey, QString* errorMessage)
{
    return openMatchedDevice(resolveCameraKey(preferredCameraKey), errorMessage);
}

bool HikCameraService::openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage)
{
    std::lock_guard<std::mutex> lifecycleLock(m_deviceLifecycleMutex);

    if (m_impl == nullptr || !m_impl->sdkReady) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("MVS SDK 尚未初始化。");
        }
        return false;
    }

    // 连接/采图线程可能并发进入：已连接则直接复用，禁止并行 close+open。
    {
        QMutexLocker locker(&m_impl->mutex);
        if (m_impl->connected && m_impl->handle != nullptr) {
            if (errorMessage) {
                errorMessage->clear();
            }
            return true;
        }
    }

    closeDeviceUnlocked();

    MV_CC_DEVICE_INFO_LIST deviceList;
    std::memset(&deviceList, 0, sizeof(deviceList));
    const int enumResult = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (enumResult != MV_OK) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("枚举海康相机失败，错误码=0x%1")
                .arg(static_cast<quint32>(enumResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    MV_CC_DEVICE_INFO* matchedDevice = nullptr;
    QString matchedIpAddress;
    QString matchedSerialNumber;
    QString matchedModelName;
    QString matchedUserDefinedName;

    for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* deviceInfo = deviceList.pDeviceInfo[i];
        if (deviceInfo == nullptr) {
            continue;
        }

        QString ipAddress;
        QString serialNumber;
        QString modelName;
        QString userDefinedName;

        if (deviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            ipAddress = ipToString(deviceInfo->SpecialInfo.stGigEInfo.nCurrentIp);
            serialNumber = trimSdkString(deviceInfo->SpecialInfo.stGigEInfo.chSerialNumber, sizeof(deviceInfo->SpecialInfo.stGigEInfo.chSerialNumber));
            modelName = trimSdkString(deviceInfo->SpecialInfo.stGigEInfo.chModelName, sizeof(deviceInfo->SpecialInfo.stGigEInfo.chModelName));
            userDefinedName = trimSdkString(deviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName, sizeof(deviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName));
        } else if (deviceInfo->nTLayerType == MV_USB_DEVICE) {
            serialNumber = trimSdkString(deviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber, sizeof(deviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
            modelName = trimSdkString(deviceInfo->SpecialInfo.stUsb3VInfo.chModelName, sizeof(deviceInfo->SpecialInfo.stUsb3VInfo.chModelName));
            userDefinedName = trimSdkString(deviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName, sizeof(deviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName));
        }

        const bool matched = preferredCameraKey.isEmpty() ||
            preferredCameraKey.compare(ipAddress, Qt::CaseInsensitive) == 0 ||
            preferredCameraKey.compare(serialNumber, Qt::CaseInsensitive) == 0 ||
            preferredCameraKey.compare(userDefinedName, Qt::CaseInsensitive) == 0 ||
            preferredCameraKey.compare(modelName, Qt::CaseInsensitive) == 0 ||
            containsKey(ipAddress, preferredCameraKey) ||
            containsKey(serialNumber, preferredCameraKey) ||
            containsKey(userDefinedName, preferredCameraKey) ||
            containsKey(modelName, preferredCameraKey);

        if (matched) {
            matchedDevice = deviceInfo;
            matchedIpAddress = ipAddress;
            matchedSerialNumber = serialNumber;
            matchedModelName = modelName;
            matchedUserDefinedName = userDefinedName;
            break;
        }
    }

    if (matchedDevice == nullptr) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("未找到匹配的海康相机：%1").arg(preferredCameraKey);
        }
        return false;
    }

    // 根据配置决定访问模式：monitor 允许与 SCMVS 共存（只读），exclusive 独占控制
    const bool isMonitor = m_endpointConfig.accessMode.compare(
        QStringLiteral("monitor"), Qt::CaseInsensitive) == 0;
    const unsigned int accessMode = isMonitor ? MV_ACCESS_Monitor : MV_ACCESS_Exclusive;

    if (!isMonitor && !MV_CC_IsDeviceAccessible(matchedDevice, MV_ACCESS_Exclusive)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("目标海康相机当前不可独占访问：%1").arg(preferredCameraKey);
        }
        return false;
    }

    void* handle = nullptr;
    const int createResult = MV_CC_CreateHandle(&handle, matchedDevice);
    if (createResult != MV_OK || handle == nullptr) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("创建海康相机句柄失败，错误码=0x%1")
                .arg(static_cast<quint32>(createResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    const int openResult = MV_CC_OpenDevice(handle, accessMode, 0);
    if (openResult != MV_OK) {
        MV_CC_DestroyHandle(handle);
        if (errorMessage) {
            *errorMessage = QStringLiteral("打开海康相机失败（模式=%1），错误码=0x%2")
                .arg(isMonitor ? QStringLiteral("monitor") : QStringLiteral("exclusive"))
                .arg(static_cast<quint32>(openResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }
    qInfo() << QStringLiteral("[%1] 相机已以 %2 模式打开")
                   .arg(m_roleName)
                   .arg(isMonitor ? QStringLiteral("monitor（只读，与SCMVS共存）")
                                  : QStringLiteral("exclusive（独占控制）"));

    if (matchedDevice->nTLayerType == MV_GIGE_DEVICE) {
        const int packetSize = MV_CC_GetOptimalPacketSize(handle);
        if (packetSize > 0) {
            MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", packetSize);
        }
    }

    // 配置相机参数（监控模式下只读，跳过所有写操作）
    if (isMonitor) {
        qInfo() << QStringLiteral("[%1] 监控模式：跳过参数写入，仅读取").arg(m_roleName);
    } else {
        int ret = 0;

        // 检查是否为读码相机（ID Reader）
        MVCC_STRINGVALUE deviceType;
        memset(&deviceType, 0, sizeof(MVCC_STRINGVALUE));
        if (MV_CC_GetStringValue(handle, "DeviceType", &deviceType) == MV_OK) {
            QString devType = QString::fromLocal8Bit(deviceType.chCurValue);
            qInfo() << "设备类型:" << devType;
            if (devType.contains("ID", Qt::CaseInsensitive) || devType.contains("Reader", Qt::CaseInsensitive)) {
                qInfo() << "检测到读码相机，使用读码模式";
            }
        }

        // 1. 设置触发模式为关闭（连续采集）
        ret = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (ret != MV_OK) {
            qWarning() << QStringLiteral("设置触发模式(TriggerMode)失败，错误码=0x") << QString::number(ret, 16);
        } else {
            qInfo() << QStringLiteral("设置 TriggerMode=0（连续采集）成功");
        }

        // 2. 尝试设置像素格式为 Mono8（读码相机可能不支持）
        ret = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080001);
        if (ret != MV_OK) {
            qWarning() << QStringLiteral("设置像素格式(PixelFormat)失败（读码相机可能不支持），错误码=0x") << QString::number(ret, 16);
        } else {
            qInfo() << QStringLiteral("设置 PixelFormat=Mono8 成功");
        }

        // 3. 设置曝光时间（微秒）
        ret = MV_CC_SetFloatValue(handle, "ExposureTime", m_exposureTimeUs);
        if (ret != MV_OK) {
            qWarning() << QStringLiteral("设置曝光(ExposureTime)失败，错误码=0x") << QString::number(ret, 16);
        } else {
            qInfo() << QStringLiteral("设置 ExposureTime=") << m_exposureTimeUs << QStringLiteral(" us 成功");
        }

        // 4. 设置增益
        ret = MV_CC_SetFloatValue(handle, "Gain", m_gain);
        if (ret != MV_OK) {
            qWarning() << QStringLiteral("设置增益(Gain)失败，错误码=0x") << QString::number(ret, 16);
        } else {
            qInfo() << QStringLiteral("设置 Gain=") << m_gain << QStringLiteral(" dB 成功");
        }

        // 5. 读取并验证当前参数
        MVCC_ENUMVALUE triggerModeValue;
        if (MV_CC_GetEnumValue(handle, "TriggerMode", &triggerModeValue) == MV_OK) {
            qInfo() << QStringLiteral("当前 TriggerMode =") << triggerModeValue.nCurValue;
        }

        MVCC_FLOATVALUE exposureValue;
        if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureValue) == MV_OK) {
            qInfo() << QStringLiteral("当前 ExposureTime =") << exposureValue.fCurValue << QStringLiteral(" us");
        }

        // 6. 检查相机是否支持 AcquisitionMode
        MVCC_ENUMVALUE acqMode;
        if (MV_CC_GetEnumValue(handle, "AcquisitionMode", &acqMode) == MV_OK) {
            qInfo() << QStringLiteral("当前 AcquisitionMode =") << acqMode.nCurValue;
            MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);  // 2=Continuous
            qInfo() << QStringLiteral("设置 AcquisitionMode=2（连续采集）");
        }

        // 7. 对于读码相机，尝试启用图像输出
        ret = MV_CC_SetBoolValue(handle, "ImageOutputEnable", true);
        if (ret == MV_OK) {
            qInfo() << "启用图像输出成功（读码相机）";
        }

        // 8. 启动采集
        ret = MV_CC_StartGrabbing(handle);
        if (ret != MV_OK) {
            qWarning() << "提前启动采集失败，错误码=0x" << QString::number(ret, 16);
        } else {
            qInfo() << "提前启动采集成功，相机开始准备图像";
            QThread::msleep(100);
            qInfo() << "等待相机稳定完成";
        }
    }

    {
        QMutexLocker locker(&m_impl->mutex);
        m_impl->handle = handle;
        m_impl->deviceInfo = *matchedDevice;
        m_impl->ipAddress = matchedIpAddress;
        m_impl->serialNumber = matchedSerialNumber;
        m_impl->modelName = matchedModelName;
        m_impl->userDefinedName = matchedUserDefinedName;
        m_impl->connected = true;
    }

    if (errorMessage) {
        errorMessage->clear();
    }
    return true;
}

// ============================================================================
// 相机参数读取
// ============================================================================

HikCameraParams HikCameraService::readParams(QString* errorMessage)
{
    HikCameraParams p;

    void* handle = nullptr;
    {
        QMutexLocker locker(&m_impl->mutex);
        if (!m_impl->connected || m_impl->handle == nullptr) {
            p.errorMessage = QStringLiteral("相机未连接，无法读取参数");
            if (errorMessage) *errorMessage = p.errorMessage;
            return p;
        }
        handle = m_impl->handle;
    }

    // ---- 辅助宏：读取失败时记录警告但继续 ----
    auto warnOnFail = [&](int ret, const char* nodeName) {
        if (ret != MV_OK) {
            qWarning() << QStringLiteral("[%1] 读取节点 %2 失败，错误码=0x%3")
                              .arg(m_roleName)
                              .arg(QString::fromLatin1(nodeName))
                              .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
        }
        return ret == MV_OK;
    };

    int ret = MV_OK;
    MVCC_FLOATVALUE fVal{};
    MVCC_ENUMVALUE eVal{};
    MVCC_INTVALUE_EX iVal{};

    // ---- 曝光时间 ----
    std::memset(&fVal, 0, sizeof(fVal));
    ret = MV_CC_GetFloatValue(handle, "ExposureTime", &fVal);
    if (warnOnFail(ret, "ExposureTime")) {
        p.exposureTimeUs    = fVal.fCurValue;
        p.exposureTimeMinUs = fVal.fMin;
        p.exposureTimeMaxUs = fVal.fMax;
    }

    // ---- 自动曝光 ----
    std::memset(&eVal, 0, sizeof(eVal));
    ret = MV_CC_GetEnumValue(handle, "ExposureAuto", &eVal);
    if (warnOnFail(ret, "ExposureAuto")) {
        p.autoExposureEnabled = (eVal.nCurValue != 0);  // 0=Off
    }

    // ---- 增益 ----
    std::memset(&fVal, 0, sizeof(fVal));
    ret = MV_CC_GetFloatValue(handle, "Gain", &fVal);
    if (warnOnFail(ret, "Gain")) {
        p.gainDb    = fVal.fCurValue;
        p.gainMinDb = fVal.fMin;
        p.gainMaxDb = fVal.fMax;
    }

    // ---- 自动增益 ----
    std::memset(&eVal, 0, sizeof(eVal));
    ret = MV_CC_GetEnumValue(handle, "GainAuto", &eVal);
    if (warnOnFail(ret, "GainAuto")) {
        p.autoGainEnabled = (eVal.nCurValue != 0);
    }

    // ---- 帧率使能 ----
    std::memset(&eVal, 0, sizeof(eVal));
    ret = MV_CC_GetEnumValue(handle, "AcquisitionFrameRateEnable", &eVal);
    if (ret == MV_OK) {
        p.frameRateEnabled = (eVal.nCurValue != 0);
    } else {
        // 部分相机用 Bool 节点，直接用 bool* 接收
        bool bVal = false;
        ret = MV_CC_GetBoolValue(handle, "AcquisitionFrameRateEnable", &bVal);
        if (ret == MV_OK) p.frameRateEnabled = bVal;
    }

    // ---- 帧率 ----
    std::memset(&fVal, 0, sizeof(fVal));
    ret = MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &fVal);
    if (warnOnFail(ret, "AcquisitionFrameRate")) {
        p.frameRateFps = fVal.fCurValue;
    }

    // ---- 触发模式 ----
    std::memset(&eVal, 0, sizeof(eVal));
    ret = MV_CC_GetEnumValue(handle, "TriggerMode", &eVal);
    if (warnOnFail(ret, "TriggerMode")) {
        p.triggerMode = eVal.nCurValue;
    }

    // ---- 图像宽高 ----
    std::memset(&iVal, 0, sizeof(iVal));
    ret = MV_CC_GetIntValueEx(handle, "Width", &iVal);
    if (warnOnFail(ret, "Width")) {
        p.width = iVal.nCurValue;
    }

    std::memset(&iVal, 0, sizeof(iVal));
    ret = MV_CC_GetIntValueEx(handle, "Height", &iVal);
    if (warnOnFail(ret, "Height")) {
        p.height = iVal.nCurValue;
    }

    // ---- 像素格式 ----
    std::memset(&eVal, 0, sizeof(eVal));
    ret = MV_CC_GetEnumValue(handle, "PixelFormat", &eVal);
    if (warnOnFail(ret, "PixelFormat")) {
        p.pixelFormat = eVal.nCurValue;
        // 读取当前枚举值对应的符号名（如 "Mono8"）
        MVCC_ENUMENTRY entry{};
        entry.nValue = eVal.nCurValue;
        if (MV_CC_GetEnumEntrySymbolic(handle, "PixelFormat", &entry) == MV_OK) {
            p.pixelFormatStr = QString::fromLatin1(entry.chSymbolic);
        } else {
            p.pixelFormatStr = QStringLiteral("0x%1").arg(eVal.nCurValue, 8, 16, QLatin1Char('0'));
        }
    }

    p.valid = true;
    if (errorMessage) errorMessage->clear();
    return p;
}

// ============================================================================
// 相机参数修改
// ============================================================================

bool HikCameraService::writeParams(const HikCameraParams& params, QString* errorMessage)
{
    void* handle = nullptr;
    {
        QMutexLocker locker(&m_impl->mutex);
        if (!m_impl->connected || m_impl->handle == nullptr) {
            if (errorMessage) *errorMessage = QStringLiteral("相机未连接，无法修改参数");
            return false;
        }
        handle = m_impl->handle;
    }

    // 监控模式下禁止写参数
    const bool isMonitor = m_endpointConfig.accessMode.compare(
        QStringLiteral("monitor"), Qt::CaseInsensitive) == 0;
    if (isMonitor) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("[%1] 监控模式下不允许写入参数，请将 accessMode 改为 exclusive")
                                .arg(m_roleName);
        }
        qWarning() << *errorMessage;
        return false;
    }

    bool allOk = true;
    int ret = MV_OK;

    // ---- 辅助：写失败时记录错误 ----
    auto checkWrite = [&](int r, const char* nodeName) {
        if (r != MV_OK) {
            const QString msg = QStringLiteral("[%1] 写入节点 %2 失败，错误码=0x%3")
                                    .arg(m_roleName)
                                    .arg(QString::fromLatin1(nodeName))
                                    .arg(static_cast<quint32>(r), 8, 16, QLatin1Char('0'));
            qWarning() << msg;
            if (errorMessage && errorMessage->isEmpty()) *errorMessage = msg;
            allOk = false;
        }
        return r == MV_OK;
    };

    // ---- 曝光时间（仅当值合理时写入）----
    if (params.exposureTimeUs > 0.0f) {
        ret = MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
        checkWrite(ret, "ExposureTime");
    }

    // ---- 自动曝光 ----
    // 0=Off, 1=Once, 2=Continuous
    ret = MV_CC_SetEnumValue(handle, "ExposureAuto", params.autoExposureEnabled ? 2u : 0u);
    checkWrite(ret, "ExposureAuto");

    // ---- 增益（仅当值合理时写入）----
    if (params.gainDb >= 0.0f) {
        ret = MV_CC_SetFloatValue(handle, "Gain", params.gainDb);
        checkWrite(ret, "Gain");
    }

    // ---- 自动增益 ----
    ret = MV_CC_SetEnumValue(handle, "GainAuto", params.autoGainEnabled ? 2u : 0u);
    checkWrite(ret, "GainAuto");

    // ---- 帧率使能 ----
    ret = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", params.frameRateEnabled);
    if (ret != MV_OK) {
        // 部分相机用枚举节点
        ret = MV_CC_SetEnumValue(handle, "AcquisitionFrameRateEnable", params.frameRateEnabled ? 1u : 0u);
        checkWrite(ret, "AcquisitionFrameRateEnable");
    }

    // ---- 帧率（仅帧率使能时有意义）----
    if (params.frameRateEnabled && params.frameRateFps > 0.0f) {
        ret = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frameRateFps);
        checkWrite(ret, "AcquisitionFrameRate");
    }

    // ---- 触发模式 ----
    ret = MV_CC_SetEnumValue(handle, "TriggerMode", params.triggerMode);
    checkWrite(ret, "TriggerMode");

    // ---- 像素格式（仅当非零时写入）----
    if (params.pixelFormat != 0) {
        ret = MV_CC_SetEnumValue(handle, "PixelFormat", params.pixelFormat);
        checkWrite(ret, "PixelFormat");
    }

    if (allOk) {
        qInfo() << QStringLiteral("[%1] 相机参数写入完成").arg(m_roleName);
        if (errorMessage) errorMessage->clear();
    }
    return allOk;
}

void HikCameraService::closeDeviceUnlocked()
{
    if (m_impl == nullptr) {
        return;
    }
    QMutexLocker locker(&m_impl->mutex);
    if (m_impl->handle != nullptr) {
        MV_CC_CloseDevice(m_impl->handle);
        MV_CC_DestroyHandle(m_impl->handle);
        m_impl->handle = nullptr;
    }
    m_impl->connected = false;
}

void HikCameraService::closeDevice()
{
    std::lock_guard<std::mutex> lifecycleLock(m_deviceLifecycleMutex);
    closeDeviceUnlocked();
}

void HikCameraService::startAsyncConnect()
{
    if (!m_started || m_connectInFlight.exchange(true)) {
        return;
    }

    const auto acceptResults = m_acceptAsyncResults;
    HikCameraService* const receiver = this;

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
                            QStringLiteral("ready"),
                            QStringLiteral("海康相机已连接：%1 (%2)")
                                .arg(receiver->m_impl->serialNumber, receiver->m_impl->ipAddress));

                        QString paramErr;
                        const HikCameraParams p = receiver->readParams(&paramErr);
                        if (p.valid) {
                            qInfo().noquote()
                                << QStringLiteral("[%1] 相机参数 | "
                                                  "曝光=%2 us (范围 %3~%4, 自动=%5) | "
                                                  "增益=%6 dB (范围 %7~%8, 自动=%9) | "
                                                  "帧率=%10 fps (使能=%11) | "
                                                  "触发=%12 | "
                                                  "分辨率=%13x%14 | "
                                                  "像素格式=%15")
                                       .arg(receiver->m_roleName)
                                       .arg(static_cast<double>(p.exposureTimeUs), 0, 'f', 1)
                                       .arg(static_cast<double>(p.exposureTimeMinUs), 0, 'f', 0)
                                       .arg(static_cast<double>(p.exposureTimeMaxUs), 0, 'f', 0)
                                       .arg(p.autoExposureEnabled ? QStringLiteral("开")
                                                                  : QStringLiteral("关"))
                                       .arg(static_cast<double>(p.gainDb), 0, 'f', 2)
                                       .arg(static_cast<double>(p.gainMinDb), 0, 'f', 1)
                                       .arg(static_cast<double>(p.gainMaxDb), 0, 'f', 1)
                                       .arg(p.autoGainEnabled ? QStringLiteral("开")
                                                              : QStringLiteral("关"))
                                       .arg(static_cast<double>(p.frameRateFps), 0, 'f', 2)
                                       .arg(p.frameRateEnabled ? QStringLiteral("开")
                                                               : QStringLiteral("关"))
                                       .arg(p.triggerMode == 0 ? QStringLiteral("连续")
                                                              : QStringLiteral("触发"))
                                       .arg(p.width)
                                       .arg(p.height)
                                       .arg(p.pixelFormatStr);
                        } else {
                            qWarning() << QStringLiteral("[%1] 读取相机参数失败：%2")
                                              .arg(receiver->m_roleName, paramErr);
                        }
                    } else {
                        const bool isMonitor = receiver->m_endpointConfig.accessMode.compare(
                            QStringLiteral("monitor"), Qt::CaseInsensitive) == 0;
                        emit receiver->stateChanged(
                            receiver->m_roleName,
                            QStringLiteral("ready"),
                            QStringLiteral("海康相机服务已启动，但尚未连接设备：%1")
                                .arg(errorMessage));
                        if (!isMonitor) {
                            emit receiver->fatalError(
                                VisionErrorCode::DeviceNotFound,
                                QStringLiteral("后台连接海康相机失败：%1").arg(cameraKey));
                        } else {
                            qWarning()
                                << QStringLiteral("[%1] 监控模式连接失败（SCMVS 可能正在独占），"
                                                  "参数读取不可用，TCP/FTP 通信不受影响：%2")
                                       .arg(receiver->m_roleName, errorMessage);
                        }
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
