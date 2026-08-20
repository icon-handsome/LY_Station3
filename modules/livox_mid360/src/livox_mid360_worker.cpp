#include "scan_tracking/livox_mid360/livox_mid360_worker.h"

#include <QtCore/QDir>
#include <QtCore/QEventLoop>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QStandardPaths>
#include <QtCore/QTimer>

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

#include <algorithm>
#include <mutex>

Q_LOGGING_CATEGORY(LOG_LIVOX_MID360, "livox.mid360")

namespace scan_tracking {
namespace livox_mid360 {

namespace {

QString logPrefix()
{
    return QStringLiteral("[LivoxMid360]");
}

QString handleToIp(quint32 handle)
{
#ifdef _WIN32
    struct in_addr addr;
    addr.S_un.S_addr = handle;
    char buffer[INET_ADDRSTRLEN] = {};
    if (inet_ntop(AF_INET, &addr, buffer, sizeof(buffer)) != nullptr) {
        return QString::fromLatin1(buffer);
    }
    return QStringLiteral("0.0.0.0");
#else
    struct in_addr addr;
    addr.s_addr = handle;
    return QString::fromLatin1(inet_ntoa(addr));
#endif
}

QString deviceTypeName(int deviceType)
{
    switch (deviceType) {
        case kLivoxLidarTypeMid360:
            return QStringLiteral("Mid-360");
        case kLivoxLidarTypeMid360s:
            return QStringLiteral("Mid-360S");
        default:
            return QStringLiteral("type=%1").arg(deviceType);
    }
}

QString formatDeviceSummaryLine(const LivoxMid360DeviceSummary& summary)
{
    return QStringLiteral("  [%1] SN=%2 lidar_ip=%3 handle_ip=%4 type=%5")
        .arg(summary.index)
        .arg(summary.serialNumber)
        .arg(summary.lidarIp)
        .arg(summary.handleIp)
        .arg(summary.deviceTypeName);
}

QString formatOpenedDeviceLine(const LivoxMid360DeviceSummary& summary)
{
    return QStringLiteral(
               "Opened device: SN=%1 lidar_ip=%2 handle_ip=%3 handle=%4 type=%5")
        .arg(summary.serialNumber)
        .arg(summary.lidarIp)
        .arg(summary.handleIp)
        .arg(summary.handle)
        .arg(summary.deviceTypeName);
}

LivoxMid360Worker* g_activeWorker = nullptr;
std::mutex g_livoxCallbackMutex;

bool hasUtf8Bom(const QByteArray& data)
{
    return data.size() >= 3
        && static_cast<unsigned char>(data[0]) == 0xEF
        && static_cast<unsigned char>(data[1]) == 0xBB
        && static_cast<unsigned char>(data[2]) == 0xBF;
}

void livoxInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
    std::lock_guard<std::mutex> lock(g_livoxCallbackMutex);
    auto* worker = static_cast<LivoxMid360Worker*>(client_data);
    if (worker == nullptr || info == nullptr || g_activeWorker != worker) {
        return;
    }

    const QString serialNumber = QString::fromLatin1(info->sn).trimmed();
    const QString lidarIp = QString::fromLatin1(info->lidar_ip).trimmed();
    const int deviceType = static_cast<int>(info->dev_type);

    QMetaObject::invokeMethod(
        worker,
        "onDeviceDiscovered",
        Qt::QueuedConnection,
        Q_ARG(quint32, handle),
        Q_ARG(QString, serialNumber),
        Q_ARG(QString, lidarIp),
        Q_ARG(int, deviceType));
}

}  // namespace

LivoxMid360Worker::LivoxMid360Worker(QObject* parent)
    : QObject(parent)
{
}

LivoxMid360Worker::~LivoxMid360Worker()
{
    stopWorker();
}

QString LivoxMid360Worker::resolveConfigPathForSdk(
    const QString& configPath,
    QString* warningMessage)
{
    QFile sourceFile(configPath);
    if (!sourceFile.open(QIODevice::ReadOnly)) {
        return configPath;
    }

    const QByteArray rawData = sourceFile.readAll();
    sourceFile.close();

    if (!hasUtf8Bom(rawData)) {
        return configPath;
    }

    cleanupTempConfigFile();
    m_tempConfigPath = QDir(QStandardPaths::writableLocation(QStandardPaths::TempLocation))
                           .filePath(QStringLiteral("scan_tracking_livox_mid360_config.json"));

    QFile tempFile(m_tempConfigPath);
    if (!tempFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (warningMessage != nullptr) {
            *warningMessage = QStringLiteral(
                "Config has UTF-8 BOM but temp file write failed; Livox SDK may reject JSON");
        }
        return configPath;
    }

    tempFile.write(rawData.mid(3));
    tempFile.close();

    if (warningMessage != nullptr) {
        *warningMessage = QStringLiteral(
            "Removed UTF-8 BOM from Livox config and using temp file: %1").arg(m_tempConfigPath);
    }
    return m_tempConfigPath;
}

void LivoxMid360Worker::cleanupTempConfigFile()
{
    if (m_tempConfigPath.isEmpty()) {
        return;
    }

    QFile::remove(m_tempConfigPath);
    m_tempConfigPath.clear();
}

void LivoxMid360Worker::startWorker(const LivoxMid360OpenConfig& config)
{
    stopWorker();

    m_openConfig = config;
    m_discoveredDevices.clear();
    m_nextDeviceIndex = 0;
    m_discoveryActive = false;

    emit logMessage(QStringLiteral("%1 Starting worker...").arg(logPrefix()));
    emit stateChanged(LivoxMid360RuntimeState::Enumerating, QStringLiteral("Enumerating devices"));

    const QString configPath = config.configFilePath.trimmed();
    if (configPath.isEmpty() || !QFile::exists(configPath)) {
        const QString message =
            QStringLiteral("Livox config file not found: %1").arg(configPath);
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        emit enumerateFinished({});
        emit openFinished(false, {}, message);
        emit stateChanged(LivoxMid360RuntimeState::Failed, message);
        return;
    }

    emit logMessage(QStringLiteral("%1 Using config: %2").arg(logPrefix(), configPath));

    QString configWarning;
    const QString sdkConfigPath = resolveConfigPathForSdk(configPath, &configWarning);
    if (!configWarning.isEmpty()) {
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), configWarning));
    }

    DisableLivoxSdkConsoleLogger();

    const QByteArray configPathBytes = QFile::encodeName(sdkConfigPath);
    if (!LivoxLidarSdkInit(configPathBytes.constData())) {
        const QString message = QStringLiteral(
            "LivoxLidarSdkInit failed for config: %1 (check JSON syntax, UTF-8 BOM, host_ip)")
            .arg(sdkConfigPath);
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        cleanupTempConfigFile();
        emit enumerateFinished({});
        emit openFinished(false, {}, message);
        emit stateChanged(LivoxMid360RuntimeState::Failed, message);
        return;
    }

    m_sdkInitialized = true;
    g_activeWorker = this;

    SetLivoxLidarInfoChangeCallback(livoxInfoChangeCallback, this);

    if (!LivoxLidarSdkStart()) {
        const QString message = QStringLiteral("LivoxLidarSdkStart failed");
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        teardownSdk();
        emit enumerateFinished({});
        emit openFinished(false, {}, message);
        emit stateChanged(LivoxMid360RuntimeState::Failed, message);
        return;
    }

    emit stateChanged(LivoxMid360RuntimeState::Opening, QStringLiteral("Waiting for device discovery"));

    m_discoveryActive = true;

    QEventLoop loop;
    QTimer discoveryTimer;
    discoveryTimer.setSingleShot(true);
    connect(&discoveryTimer, &QTimer::timeout, this, &LivoxMid360Worker::onDiscoveryTimeout);
    connect(
        this,
        &LivoxMid360Worker::discoveryWindowFinished,
        &loop,
        &QEventLoop::quit,
        Qt::QueuedConnection);

    const int timeoutMs = config.discoveryTimeoutMs > 0 ? config.discoveryTimeoutMs : 10000;
    discoveryTimer.start(timeoutMs);
    loop.exec();

    finishDiscovery();
}

void LivoxMid360Worker::onDeviceDiscovered(
    quint32 handle,
    QString serialNumber,
    QString lidarIp,
    int deviceType)
{
    if (!m_discoveryActive) {
        return;
    }

    const auto existingIt = std::find_if(
        m_discoveredDevices.begin(),
        m_discoveredDevices.end(),
        [handle](const LivoxMid360DeviceSummary& item) {
            return item.handle == handle;
        });
    if (existingIt != m_discoveredDevices.end()) {
        return;
    }

    LivoxMid360DeviceSummary summary;
    summary.index = m_nextDeviceIndex++;
    summary.handle = handle;
    summary.serialNumber = serialNumber;
    summary.lidarIp = lidarIp;
    summary.handleIp = handleToIp(handle);
    summary.deviceType = deviceType;
    summary.deviceTypeName = deviceTypeName(deviceType);
    m_discoveredDevices.push_back(summary);

    emit logMessage(
        QStringLiteral("%1 %2").arg(logPrefix(), formatDeviceSummaryLine(summary)));

    const QString trimmedSerial = m_openConfig.serial.trimmed();
    if (!trimmedSerial.isEmpty() && serialNumber.compare(trimmedSerial, Qt::CaseInsensitive) == 0) {
        emit discoveryWindowFinished();
    }
}

void LivoxMid360Worker::onDiscoveryTimeout()
{
    if (!m_discoveryActive) {
        return;
    }
    emit discoveryWindowFinished();
}

void LivoxMid360Worker::finishDiscovery()
{
    if (!m_discoveryActive) {
        return;
    }
    m_discoveryActive = false;

    emit logMessage(
        QStringLiteral("%1 Enumerated %2 device(s)")
            .arg(logPrefix())
            .arg(m_discoveredDevices.size()));
    emit enumerateFinished(m_discoveredDevices);

    const QString trimmedSerial = m_openConfig.serial.trimmed();
    LivoxMid360DeviceSummary selectedDevice;
    bool hasSelectedDevice = false;

    if (!trimmedSerial.isEmpty()) {
        const auto matchedIt = std::find_if(
            m_discoveredDevices.begin(),
            m_discoveredDevices.end(),
            [&trimmedSerial](const LivoxMid360DeviceSummary& item) {
                return item.serialNumber.compare(trimmedSerial, Qt::CaseInsensitive) == 0;
            });
        if (matchedIt != m_discoveredDevices.end()) {
            selectedDevice = *matchedIt;
            hasSelectedDevice = true;
        }
    } else if (!m_discoveredDevices.isEmpty()) {
        selectedDevice = m_discoveredDevices.front();
        hasSelectedDevice = true;
    }

    if (!hasSelectedDevice) {
        const QString message = trimmedSerial.isEmpty()
            ? QStringLiteral("No Livox Mid-360 device discovered")
            : QStringLiteral("No Livox device matched serial: %1").arg(trimmedSerial);
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        emit openFinished(false, {}, message);
        emit stateChanged(LivoxMid360RuntimeState::Failed, message);
        {
            std::lock_guard<std::mutex> lock(g_livoxCallbackMutex);
            if (g_activeWorker == this) {
                g_activeWorker = nullptr;
            }
            SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
        }
        m_discoveryActive = false;
        // 发现失败时不调用 LivoxLidarSdkUninit：现场/联调环境常无雷达，Uninit 易与 SDK 后台线程竞态崩溃。
        m_sdkInitialized = false;
        return;
    }

    emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), formatOpenedDeviceLine(selectedDevice)));
    emit logMessage(QStringLiteral("%1 Ready (no point cloud stream started)").arg(logPrefix()));
    emit openFinished(true, selectedDevice, {});
    emit stateChanged(LivoxMid360RuntimeState::Ready, QStringLiteral("Device opened"));
}

void LivoxMid360Worker::teardownSdk()
{
    {
        std::lock_guard<std::mutex> lock(g_livoxCallbackMutex);
        if (g_activeWorker == this) {
            g_activeWorker = nullptr;
        }
        SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
    }

    if (m_sdkInitialized) {
        LivoxLidarSdkUninit();
        m_sdkInitialized = false;
    }

    cleanupTempConfigFile();
}

void LivoxMid360Worker::stopWorker()
{
    const bool wasActive = m_sdkInitialized || m_discoveryActive;
    m_discoveryActive = false;
    teardownSdk();
    m_discoveredDevices.clear();
    m_nextDeviceIndex = 0;
    if (wasActive) {
        emit stateChanged(LivoxMid360RuntimeState::Stopped, QStringLiteral("Worker stopped"));
    }
}

}  // namespace livox_mid360
}  // namespace scan_tracking
