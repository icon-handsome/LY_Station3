#include "scan_tracking/vision/hik_camera_c_controller.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>
#include <QtCore/QTimer>

#include "scan_tracking/vision/hik_smart_camera_tcp_server.h"
#include "scan_tracking/vision/hik_smart_camera_ftp_monitor.h"

Q_LOGGING_CATEGORY(hikCControllerLog, "vision.hik_camera_c_controller")

namespace scan_tracking {
namespace vision {

void HikCameraCController::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::HikCameraCState>("scan_tracking::vision::HikCameraCState");
    qRegisterMetaType<scan_tracking::vision::CaptureType>("scan_tracking::vision::CaptureType");
    qRegisterMetaType<scan_tracking::vision::ImageFileInfo>("scan_tracking::vision::ImageFileInfo");
    registered = true;
}

HikCameraCController::HikCameraCController(QObject* parent)
    : QObject(parent)
    , m_tcpServer(nullptr)
    , m_testCaptureTimer(nullptr)
{
    registerMetaTypes();
}

HikCameraCController::~HikCameraCController()
{
    cleanupFtpMonitors();
    cleanupTcpServer();

    if (m_testCaptureTimer) {
        m_testCaptureTimer->stop();
        m_testCaptureTimer->setParent(nullptr);
        delete m_testCaptureTimer;
        m_testCaptureTimer = nullptr;
    }
}

void HikCameraCController::start(const scan_tracking::common::VisionConfig& config)
{
    if (m_started) {
        qWarning(hikCControllerLog) << QStringLiteral("HikCameraCController 已启动，无需重复启动。");
        return;
    }

    m_config = config;
    m_configuredCameraIps.clear();

    const auto appendGroup = [this](const scan_tracking::common::VisionDeviceGroupConfig& group) {
        const QString ip = group.hikCameraC.ipAddress.trimmed();
        if (!ip.isEmpty() && !m_configuredCameraIps.contains(ip)) {
            m_configuredCameraIps.append(ip);
        }
    };
    appendGroup(config.telescopicGroup);
    appendGroup(config.armGroup);
    if (m_configuredCameraIps.isEmpty()) {
        const QString legacyIp = config.hikCameraC.ipAddress.trimmed();
        if (!legacyIp.isEmpty()) {
            m_configuredCameraIps.append(legacyIp);
        }
    }

    m_primaryCameraIp = m_configuredCameraIps.isEmpty()
                            ? config.telescopicGroup.hikCameraC.ipAddress
                            : m_configuredCameraIps.first();
    m_tcpListenIp = config.hikCameraCTcpListenIp.isEmpty()
                        ? QStringLiteral("192.168.8.13")
                        : config.hikCameraCTcpListenIp;
    m_tcpListenPort = config.hikCameraCTcpListenPort > 0
                          ? config.hikCameraCTcpListenPort
                          : 8999;

    m_started = true;
    setState(HikCameraCState::Initializing, QStringLiteral("海康相机 C 控制器正在初始化（纯TCP模式）"));

    qInfo(hikCControllerLog).noquote()
        << QStringLiteral("HikCameraCController 已启动，期望相机：")
        << formatConfiguredCameraList();

    initializeTcpServer();
    initializeFtpMonitors();
    scheduleStartupConnectionReport();
}

void HikCameraCController::stop()
{
    if (!m_started) {
        return;
    }

    if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
        m_testCaptureTimer->stop();
        qInfo(hikCControllerLog) << "测试采集定时器已停止";
    }

    m_started = false;
    cleanupFtpMonitors();
    cleanupTcpServer();
    setState(HikCameraCState::Stopped, QStringLiteral("海康相机 C 控制器已停止"));
    qInfo(hikCControllerLog) << QStringLiteral("HikCameraCController 已停止。");
}

void HikCameraCController::initializeTcpServer()
{
    if (m_tcpServer != nullptr) {
        qWarning(hikCControllerLog) << QStringLiteral("TCP 服务器已初始化");
        return;
    }

    m_tcpServer = new HikSmartCameraTcpServer(this);

    connect(m_tcpServer, &HikSmartCameraTcpServer::serverStarted,
            this, &HikCameraCController::onTcpServerStarted);
    connect(m_tcpServer, &HikSmartCameraTcpServer::serverStopped,
            this, &HikCameraCController::onTcpServerStopped);
    // 相机可能在 start() 栈内立即连入；QueuedConnection 避免 initModules 尚未完成时重入 setState。
    connect(m_tcpServer, &HikSmartCameraTcpServer::cameraConnected,
            this, &HikCameraCController::onTcpCameraConnected,
            Qt::QueuedConnection);
    connect(m_tcpServer, &HikSmartCameraTcpServer::cameraDisconnected,
            this, &HikCameraCController::onTcpCameraDisconnected,
            Qt::QueuedConnection);
    connect(m_tcpServer, &HikSmartCameraTcpServer::heartbeatReceived,
            this, &HikCameraCController::onTcpHeartbeatReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::commandReceived,
            this, &HikCameraCController::onTcpCommandReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::imageDataReceived,
            this, &HikCameraCController::onTcpImageDataReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::error,
            this, &HikCameraCController::onTcpError);

    const QString listenIp = m_tcpListenIp;
    const quint16 listenPort = m_tcpListenPort;

    if (!m_tcpServer->start(listenIp, listenPort)) {
        qWarning(hikCControllerLog) << QStringLiteral("首次启动 TCP 服务器失败，等待 2 秒后重试...");
        QThread::msleep(2000);

        if (!m_tcpServer->start(listenIp, listenPort)) {
            qCritical(hikCControllerLog) << QStringLiteral("重试后仍无法启动 TCP 服务器") << listenIp << QStringLiteral(":") << listenPort;
            setState(HikCameraCState::Error, QStringLiteral("TCP 服务器启动失败（端口可能被占用）"));
            emit fatalError(VisionErrorCode::DeviceOpenFailed,
                          QStringLiteral("TCP 服务器启动失败，端口 %1 可能被其他进程占用。").arg(listenPort));
        } else {
            qInfo(hikCControllerLog) << "重试后 TCP 服务器启动成功";
        }
    } else {
        qInfo(hikCControllerLog) << "TCP 服务器启动成功，地址" << listenIp << ":" << listenPort;
    }
}

void HikCameraCController::cleanupTcpServer()
{
    if (m_tcpServer != nullptr) {
        m_tcpServer->stop();
        m_tcpServer->setParent(nullptr);
        delete m_tcpServer;
        m_tcpServer = nullptr;
        qInfo(hikCControllerLog) << "TCP 服务器已清理";
    }
}

void HikCameraCController::initializeFtpMonitors()
{
    if (!m_ftpBindings.isEmpty()) {
        qWarning(hikCControllerLog) << QStringLiteral("FTP 监控器已初始化");
        return;
    }

    const auto addBinding = [this](const scan_tracking::common::VisionDeviceGroupConfig& group) {
        const QString ip = group.hikCameraC.ipAddress.trimmed();
        const QString dir = group.hikCameraCFtpDirectory.trimmed();
        if (ip.isEmpty() || dir.isEmpty()) {
            return;
        }
        for (const FtpBinding& existing : m_ftpBindings) {
            if (existing.cameraIp == ip || existing.ftpDirectory == dir) {
                return;
            }
        }

        FtpBinding binding;
        binding.cameraIp = ip;
        binding.ftpDirectory = dir;
        binding.monitor = new HikSmartCameraFtpMonitor(this);

        connect(binding.monitor, &HikSmartCameraFtpMonitor::monitorStarted,
                this, &HikCameraCController::onFtpMonitorStarted);
        connect(binding.monitor, &HikSmartCameraFtpMonitor::monitorStopped,
                this, &HikCameraCController::onFtpMonitorStopped);
        connect(binding.monitor, &HikSmartCameraFtpMonitor::newImageDetected,
                this, &HikCameraCController::onFtpNewImageDetected);
        connect(binding.monitor, &HikSmartCameraFtpMonitor::imageReady,
                this, &HikCameraCController::onFtpImageReady);
        connect(binding.monitor, &HikSmartCameraFtpMonitor::error,
                this, &HikCameraCController::onFtpError);

        if (!binding.monitor->start(dir)) {
            qWarning(hikCControllerLog).noquote()
                << QStringLiteral("FTP 监控器启动失败，相机=") << ip
                << QStringLiteral(" 目录=") << dir
                << QStringLiteral("（海康 C 仍可通过 TCP 通信）");
        } else {
            qInfo(hikCControllerLog).noquote()
                << QStringLiteral("FTP 监控器启动成功，相机=") << ip
                << QStringLiteral(" 目录=") << dir;
        }
        m_ftpBindings.append(binding);
    };

    addBinding(m_config.telescopicGroup);
    addBinding(m_config.armGroup);

    if (m_ftpBindings.isEmpty()) {
        const QString legacyDir = m_config.hikCameraCFtpDirectory.trimmed();
        if (!legacyDir.isEmpty() && !m_primaryCameraIp.isEmpty()) {
            scan_tracking::common::VisionDeviceGroupConfig legacyGroup;
            legacyGroup.hikCameraC.ipAddress = m_primaryCameraIp;
            legacyGroup.hikCameraCFtpDirectory = legacyDir;
            addBinding(legacyGroup);
        }
    }
}

void HikCameraCController::cleanupFtpMonitors()
{
    for (FtpBinding& binding : m_ftpBindings) {
        if (binding.monitor != nullptr) {
            binding.monitor->stop();
            binding.monitor->setParent(nullptr);
            delete binding.monitor;
            binding.monitor = nullptr;
        }
    }
    m_ftpBindings.clear();
    qInfo(hikCControllerLog) << "FTP 监控器已清理";
}

bool HikCameraCController::isTcpServerRunning() const
{
    return m_tcpServer != nullptr && m_tcpServer->isListening();
}

bool HikCameraCController::isCameraConnected(const QString& cameraIp) const
{
    if (m_tcpServer == nullptr || cameraIp.trimmed().isEmpty()) {
        return false;
    }
    return m_tcpServer->connectedCameras().contains(cameraIp.trimmed());
}

bool HikCameraCController::isCameraConnectedToTcp() const
{
    return isCameraConnected(m_primaryCameraIp);
}

bool HikCameraCController::isFtpMonitorRunning() const
{
    for (const FtpBinding& binding : m_ftpBindings) {
        if (binding.monitor != nullptr && binding.monitor->isMonitoring()) {
            return true;
        }
    }
    return false;
}

QString HikCameraCController::ftpDirectory() const
{
    if (m_ftpBindings.isEmpty()) {
        return QString();
    }
    return m_ftpBindings.first().ftpDirectory;
}

QStringList HikCameraCController::configuredCameraIps() const
{
    return m_configuredCameraIps;
}

bool HikCameraCController::requestCapture(CaptureType type)
{
    return requestCapture(type, m_primaryCameraIp);
}

bool HikCameraCController::requestCapture(CaptureType type, const QString& cameraIp)
{
    const QString normalizedIp = cameraIp.trimmed();
    if (!isTcpServerRunning()) {
        qWarning(hikCControllerLog) << QStringLiteral("无法请求采集：TCP 服务器未运行");
        return false;
    }

    const QString label = groupLabelForCamera(normalizedIp);
    if (!isCameraConnected(normalizedIp)) {
        qWarning(hikCControllerLog).noquote()
            << QStringLiteral("无法请求采集：") << label << normalizedIp
            << QStringLiteral(" 未通过 TCP 连接");
        return false;
    }

    m_pendingCaptureTypeByIp[normalizedIp] = type;
    m_captureCounterByIp[normalizedIp] = m_captureCounterByIp.value(normalizedIp, 0) + 1;

    qInfo(hikCControllerLog).noquote()
        << QStringLiteral("请求采集 #") << m_captureCounterByIp.value(normalizedIp)
        << label << normalizedIp
        << QStringLiteral(" 类型：") << getCaptureTypeString(type);

    return m_tcpServer->sendStartCaptureToCamera(normalizedIp);
}

void HikCameraCController::enableTestMode(bool enable, int intervalMs)
{
    if (enable) {
        if (m_testCaptureTimer == nullptr) {
            m_testCaptureTimer = new QTimer(this);
            connect(m_testCaptureTimer, &QTimer::timeout,
                    this, &HikCameraCController::onTestCaptureTimer);
        }

        m_testCaptureTimer->setInterval(intervalMs);
        m_testCaptureTimer->start();
        qInfo(hikCControllerLog) << "测试模式已启用，采集间隔：" << intervalMs << "ms";
    } else if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
        m_testCaptureTimer->stop();
        qInfo(hikCControllerLog) << QStringLiteral("测试模式已禁用");
    }
}

QString HikCameraCController::getCaptureTypeString(CaptureType type) const
{
    switch (type) {
    case CaptureType::SurfaceDefect:
        return QStringLiteral("SurfaceDefect");
    case CaptureType::WeldDefect:
        return QStringLiteral("WeldDefect");
    case CaptureType::NumberRecognition:
        return QStringLiteral("NumberRecognition");
    default:
        return QStringLiteral("Unknown");
    }
}

bool HikCameraCController::saveImageToFile(
    const QByteArray& imageData,
    CaptureType type,
    const QString& cameraIp)
{
    QString saveDir = QStringLiteral("./smart_camera_images");
    QDir dir;
    if (!dir.exists(saveDir)) {
        if (!dir.mkpath(saveDir)) {
            qWarning(hikCControllerLog) << QStringLiteral("创建目录失败：") << saveDir;
            return false;
        }
    }

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString typeStr = getCaptureTypeString(type);
    const int counter = m_captureCounterByIp.value(cameraIp.trimmed(), 0);
    const QString filename = QStringLiteral("%1/%2_%3_%4_%5.jpg")
                                 .arg(saveDir)
                                 .arg(typeStr)
                                 .arg(cameraIp.trimmed().replace(QLatin1Char('.'), QLatin1Char('_')))
                                 .arg(timestamp)
                                 .arg(counter, 4, 10, QChar('0'));

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning(hikCControllerLog) << QStringLiteral("打开文件写入失败：") << filename;
        return false;
    }

    const qint64 written = file.write(imageData);
    file.close();

    if (written != imageData.size()) {
        qWarning(hikCControllerLog) << QStringLiteral("图像数据写入不完整：") << filename;
        return false;
    }

    qInfo(hikCControllerLog) << QStringLiteral("图像保存成功：") << filename
                             << QStringLiteral(" size=") << imageData.size() << QStringLiteral(" bytes");
    return true;
}

void HikCameraCController::setState(HikCameraCState state, const QString& description)
{
    if (m_state != state) {
        m_state = state;
        qInfo(hikCControllerLog) << QStringLiteral("状态切换为") << static_cast<int>(state) << QStringLiteral("：") << description;
        emit stateChanged(state, description);
    }
}

bool HikCameraCController::isConfiguredCameraIp(const QString& cameraIp) const
{
    return m_configuredCameraIps.contains(cameraIp.trimmed());
}

QString HikCameraCController::groupLabelForCamera(const QString& cameraIp) const
{
    const QString ip = cameraIp.trimmed();
    if (ip == m_config.telescopicGroup.hikCameraC.ipAddress.trimmed()) {
        return QStringLiteral("[海康C-伸缩杆]");
    }
    if (ip == m_config.armGroup.hikCameraC.ipAddress.trimmed()) {
        return QStringLiteral("[海康C-机械臂]");
    }
    return QStringLiteral("[海康C]");
}

QString HikCameraCController::formatConfiguredCameraList() const
{
    QStringList parts;
    for (const QString& ip : m_configuredCameraIps) {
        parts.append(QStringLiteral("%1 %2").arg(groupLabelForCamera(ip), ip));
    }
    return parts.join(QLatin1String(", "));
}

void HikCameraCController::reportConfiguredCameraConnections() const
{
    if (!m_started || m_configuredCameraIps.isEmpty()) {
        return;
    }

    for (const QString& ip : m_configuredCameraIps) {
        const QString label = groupLabelForCamera(ip);
        if (isCameraConnected(ip)) {
            qInfo(hikCControllerLog).noquote()
                << label << ip << QStringLiteral("启动检查：TCP 已连接");
        } else {
            qWarning(hikCControllerLog).noquote()
                << label << ip << QStringLiteral("启动检查：TCP 仍未连接");
        }
    }
}

void HikCameraCController::scheduleStartupConnectionReport()
{
    constexpr int kStartupConnectionReportDelayMs = 4000;
    QTimer::singleShot(kStartupConnectionReportDelayMs, this, [this]() {
        if (m_started) {
            reportConfiguredCameraConnections();
        }
    });
}

void HikCameraCController::scheduleReadyStateUpdate()
{
    QTimer::singleShot(0, this, [this]() {
        if (m_started) {
            updateReadyStateFromConnections();
        }
    });
}

void HikCameraCController::updateReadyStateFromConnections()
{
    if (!m_started || m_tcpServer == nullptr) {
        return;
    }

    QStringList connectedConfigured;
    for (const QString& ip : m_configuredCameraIps) {
        if (isCameraConnected(ip)) {
            connectedConfigured.append(
                QStringLiteral("%1 %2").arg(groupLabelForCamera(ip), ip));
        }
    }

    if (connectedConfigured.isEmpty()) {
        return;
    }

    setState(
        HikCameraCState::Ready,
        QStringLiteral("已连接智能相机：%1").arg(connectedConfigured.join(QLatin1String(", "))));
}

HikCameraCController::FtpBinding* HikCameraCController::findFtpBindingByMonitor(
    HikSmartCameraFtpMonitor* monitor)
{
    for (FtpBinding& binding : m_ftpBindings) {
        if (binding.monitor == monitor) {
            return &binding;
        }
    }
    return nullptr;
}

void HikCameraCController::onTcpServerStarted(QString listenIp, quint16 port)
{
    qInfo(hikCControllerLog) << "TCP 服务器已启动，地址" << listenIp << ":" << port;
    qInfo(hikCControllerLog).noquote()
        << QStringLiteral("等待智能相机连接：") << formatConfiguredCameraList();
}

void HikCameraCController::onTcpServerStopped()
{
    qInfo(hikCControllerLog) << "TCP 服务器已停止";
}

void HikCameraCController::onTcpCameraConnected(QString cameraIp, quint16 cameraPort)
{
    const QString label = groupLabelForCamera(cameraIp);
    qInfo(hikCControllerLog).noquote()
        << label << cameraIp << QStringLiteral(":") << cameraPort
        << QStringLiteral(" TCP 已连接");

    if (isConfiguredCameraIp(cameraIp)) {
        scheduleReadyStateUpdate();

        if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
            m_testCaptureTimer->stop();
        }
        qInfo(hikCControllerLog).noquote()
            << label << cameraIp << QStringLiteral(" 已就绪，等待 PLC 触发后再发送 start");
    } else {
        qWarning(hikCControllerLog).noquote()
            << QStringLiteral("未配置相机 IP 已连接：") << cameraIp
            << QStringLiteral("（期望：") << formatConfiguredCameraList()
            << QStringLiteral("）");
    }
}

void HikCameraCController::onTcpCameraDisconnected(QString cameraIp)
{
    const QString label = groupLabelForCamera(cameraIp);
    qWarning(hikCControllerLog).noquote()
        << label << cameraIp << QStringLiteral(" TCP 已断开");

    if (isConfiguredCameraIp(cameraIp)) {
        if (m_started && m_state == HikCameraCState::Ready) {
            qInfo(hikCControllerLog).noquote()
                << label << cameraIp << QStringLiteral(" TCP 会话结束，等待重连");
        }

        if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
            m_testCaptureTimer->stop();
            qInfo(hikCControllerLog) << "因断开连接，测试采集定时器已停止";
        }
    }
}

void HikCameraCController::onTcpHeartbeatReceived(QString cameraIp)
{
    Q_UNUSED(cameraIp);
}

void HikCameraCController::onTcpCommandReceived(QString cameraIp, QString command)
{
    QString ocrText = command.trimmed();
    if (ocrText.endsWith(QLatin1Char(';'))) {
        ocrText.chop(1);
    }

    if (ocrText.startsWith(QStringLiteral("PX"), Qt::CaseInsensitive)) {
        ocrText = ocrText.mid(2);
    } else if (ocrText.startsWith(QLatin1Char('X'), Qt::CaseInsensitive)) {
        ocrText = ocrText.mid(1);
    }

    if (!ocrText.isEmpty()
        && ocrText.compare(QStringLiteral("hello"), Qt::CaseInsensitive) != 0
        && ocrText.compare(QStringLiteral("heartbeat"), Qt::CaseInsensitive) != 0) {
        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();

        // 先通知业务层，再做日志节流（避免抑制影响读码握手）。
        m_lastOcrText = ocrText;
        m_lastOcrCameraIp = cameraIp.trimmed();
        emit ocrTextReceived(m_lastOcrCameraIp, ocrText);

        const qint64 sinceLastLogMs = (m_lastOcrLogMs > 0) ? (nowMs - m_lastOcrLogMs) : INT64_MAX;
        const bool sameAsLastLogged =
            !m_lastLoggedOcrText.isEmpty() && ocrText == m_lastLoggedOcrText;
        if ((sameAsLastLogged && sinceLastLogMs < 2000) || sinceLastLogMs < 300) {
            ++m_suppressedOcrLogCount;
            return;
        }

        if (m_suppressedOcrLogCount > 0) {
            qInfo(hikCControllerLog).noquote()
                << QStringLiteral("[HikCameraC][OCR][%1] %2（已抑制 %3 条过快/重复回包）")
                       .arg(cameraIp, ocrText)
                       .arg(m_suppressedOcrLogCount);
            m_suppressedOcrLogCount = 0;
        } else {
            qInfo(hikCControllerLog).noquote()
                << QStringLiteral("[HikCameraC][OCR][%1] %2").arg(cameraIp, ocrText);
        }

        m_lastLoggedOcrText = ocrText;
        m_lastOcrLogMs = nowMs;
        return;
    }

    if (ocrText.compare(QStringLiteral("hello"), Qt::CaseInsensitive) != 0 &&
        ocrText.compare(QStringLiteral("heartbeat"), Qt::CaseInsensitive) != 0) {
        qDebug(hikCControllerLog).noquote()
            << QStringLiteral("[HikCameraC][TCP] %1: %2").arg(cameraIp, ocrText);
    }
}

void HikCameraCController::onTcpImageDataReceived(QString cameraIp, QByteArray imageData)
{
    const CaptureType type = m_pendingCaptureTypeByIp.value(
        cameraIp.trimmed(),
        CaptureType::SurfaceDefect);

    qInfo(hikCControllerLog).noquote()
        << QStringLiteral("收到图像数据：") << cameraIp
        << QStringLiteral(" size=") << imageData.size() << QStringLiteral(" bytes")
        << QStringLiteral(" type=") << getCaptureTypeString(type);

    if (saveImageToFile(imageData, type, cameraIp)) {
        qInfo(hikCControllerLog) << QStringLiteral("图像保存成功");
    } else {
        qWarning(hikCControllerLog) << QStringLiteral("图像保存失败");
    }

    emit captureCompleted(type, cameraIp, imageData);
}

void HikCameraCController::onTcpError(QString errorMessage)
{
    qWarning(hikCControllerLog) << QStringLiteral("TCP 提示：") << errorMessage;
    if (errorMessage.contains(QStringLiteral("心跳超时"))) {
        return;
    }
    emit fatalError(VisionErrorCode::DeviceOpenFailed, errorMessage);
}

void HikCameraCController::onTestCaptureTimer()
{
    if (!m_started || m_configuredCameraIps.isEmpty()) {
        qWarning(hikCControllerLog) << QStringLiteral("测试采集跳过：未就绪");
        return;
    }

    for (const QString& ip : m_configuredCameraIps) {
        if (isCameraConnected(ip)) {
            qInfo(hikCControllerLog).noquote()
                << QStringLiteral("测试采集已触发，相机=") << ip;
            requestCapture(CaptureType::NumberRecognition, ip);
        }
    }
}

void HikCameraCController::onFtpMonitorStarted(QString directory)
{
    qInfo(hikCControllerLog) << "FTP 监控器已启动，监控目录：" << directory;
}

void HikCameraCController::onFtpMonitorStopped()
{
    qInfo(hikCControllerLog) << "FTP 监控器已停止";
}

void HikCameraCController::onFtpNewImageDetected(ImageFileInfo imageInfo)
{
    Q_UNUSED(imageInfo);
}

void HikCameraCController::onFtpImageReady(ImageFileInfo imageInfo)
{
    const auto* senderMonitor = qobject_cast<HikSmartCameraFtpMonitor*>(sender());
    const FtpBinding* binding = findFtpBindingByMonitor(
        const_cast<HikSmartCameraFtpMonitor*>(senderMonitor));
    const QString cameraIp = binding != nullptr ? binding->cameraIp : m_primaryCameraIp;

    emit imageReceived(imageInfo.captureType, cameraIp, imageInfo.filePath, imageInfo.fileSize);
}

void HikCameraCController::onFtpError(QString errorMessage)
{
    qWarning(hikCControllerLog) << QStringLiteral("FTP 监控器错误：") << errorMessage;
}

}  // namespace vision
}  // namespace scan_tracking
