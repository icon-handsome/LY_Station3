#include "scan_tracking/livox_mid360/livox_mid360_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QThread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/livox_mid360/livox_mid360_worker.h"

Q_LOGGING_CATEGORY(LOG_LIVOX_MID360_SVC, "livox.mid360.service")

namespace scan_tracking {
namespace livox_mid360 {

namespace {

QString resolveSdkRoot(const common::LivoxMid360Config& config, const QString& configIniPath)
{
    const QString rawRoot = config.sdkRoot.trimmed();
    if (rawRoot.isEmpty()) {
        return QString();
    }

    if (QFileInfo(rawRoot).isAbsolute()) {
        return QDir::cleanPath(rawRoot);
    }

    const QString configDirRoot =
        QDir::cleanPath(QFileInfo(configIniPath).absoluteDir().filePath(rawRoot));
    if (QFileInfo::exists(configDirRoot)) {
        return configDirRoot;
    }

    const QString exeDirRoot =
        QDir::cleanPath(QDir(QCoreApplication::applicationDirPath()).filePath(rawRoot));
    if (QFileInfo::exists(exeDirRoot)) {
        return exeDirRoot;
    }

    return configDirRoot;
}

QString resolveLivoxConfigFilePath(
    const common::LivoxMid360Config& config,
    const QString& configIniPath)
{
    QString relativePath = config.configFile.trimmed();
    if (relativePath.isEmpty()) {
        relativePath = QStringLiteral("bin/mid360_config.json");
    }

    const QFileInfo directInfo(relativePath);
    if (directInfo.isAbsolute()) {
        return QDir::cleanPath(relativePath);
    }

    const QString sdkRoot = resolveSdkRoot(config, configIniPath);
    if (!sdkRoot.isEmpty()) {
        const QString sdkPath = QDir(sdkRoot).filePath(relativePath);
        if (QFileInfo::exists(sdkPath)) {
            return QDir::cleanPath(sdkPath);
        }
        return QDir::cleanPath(sdkPath);
    }

    const QString configDirPath =
        QFileInfo(configIniPath).absoluteDir().filePath(relativePath);
    if (QFileInfo::exists(configDirPath)) {
        return QDir::cleanPath(configDirPath);
    }

    const QString exeDirPath =
        QDir(QCoreApplication::applicationDirPath()).filePath(relativePath);
    if (QFileInfo::exists(exeDirPath)) {
        return QDir::cleanPath(exeDirPath);
    }

    if (!sdkRoot.isEmpty()) {
        return QDir::cleanPath(QDir(sdkRoot).filePath(relativePath));
    }

    return QDir::cleanPath(configDirPath);
}

}  // namespace

void LivoxMid360Service::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    qRegisterMetaType<scan_tracking::livox_mid360::LivoxMid360RuntimeState>(
        "scan_tracking::livox_mid360::LivoxMid360RuntimeState");
    qRegisterMetaType<scan_tracking::livox_mid360::LivoxMid360OpenConfig>(
        "scan_tracking::livox_mid360::LivoxMid360OpenConfig");
    qRegisterMetaType<scan_tracking::livox_mid360::LivoxMid360DeviceSummary>(
        "scan_tracking::livox_mid360::LivoxMid360DeviceSummary");
    qRegisterMetaType<QVector<scan_tracking::livox_mid360::LivoxMid360DeviceSummary>>(
        "QVector<scan_tracking::livox_mid360::LivoxMid360DeviceSummary>");
    registered = true;
}

LivoxMid360Service::LivoxMid360Service(QObject* parent)
    : QObject(parent)
{
}

LivoxMid360Service::~LivoxMid360Service()
{
    stop();
}

void LivoxMid360Service::start()
{
    if (m_started) {
        return;
    }

    registerMetaTypes();

    const auto* configManager = common::ConfigManager::instance();
    if (configManager != nullptr) {
        const auto& config = configManager->livoxMid360Config();
        m_openConfig.serial = config.serial;
        m_openConfig.discoveryTimeoutMs = config.discoveryTimeoutMs;
        m_openConfig.configFilePath = resolveLivoxConfigFilePath(
            config,
            configManager->configFilePath());
        if (!m_openConfig.configFilePath.isEmpty()) {
            qInfo(LOG_LIVOX_MID360_SVC).noquote()
                << QStringLiteral("[LivoxMid360] Resolved config:")
                << m_openConfig.configFilePath;
        }
    }

    m_workerThread = new QThread();
    m_worker = new LivoxMid360Worker();
    m_worker->moveToThread(m_workerThread);

    connect(this, &LivoxMid360Service::sig_startWorker,
            m_worker, &LivoxMid360Worker::startWorker, Qt::QueuedConnection);
    connect(this, &LivoxMid360Service::sig_stopWorker,
            m_worker, &LivoxMid360Worker::stopWorker, Qt::QueuedConnection);

    connect(m_worker, &LivoxMid360Worker::enumerateFinished,
            this, &LivoxMid360Service::onWorkerEnumerateFinished, Qt::QueuedConnection);
    connect(m_worker, &LivoxMid360Worker::openFinished,
            this, &LivoxMid360Service::onWorkerOpenFinished, Qt::QueuedConnection);
    connect(m_worker, &LivoxMid360Worker::stateChanged,
            this, &LivoxMid360Service::onWorkerStateChanged, Qt::QueuedConnection);
    connect(m_worker, &LivoxMid360Worker::logMessage,
            this, &LivoxMid360Service::onWorkerLogMessage, Qt::QueuedConnection);

    m_workerThread->setObjectName(QStringLiteral("LivoxMid360WorkerThread"));
    m_workerThread->start();

    m_started = true;
    m_stopping = false;
    m_currentState = LivoxMid360RuntimeState::Idle;

    emit sig_startWorker(m_openConfig);
}

void LivoxMid360Service::stop()
{
    if (!m_started) {
        return;
    }

    m_stopping = true;

    // 与 Orbbec/MechEye 一致：阻塞释放 SDK → 同线程 deleteLater → quit/wait。
    if (m_worker != nullptr && m_workerThread != nullptr && m_workerThread->isRunning()) {
        QMetaObject::invokeMethod(
            m_worker,
            "stopWorker",
            Qt::BlockingQueuedConnection);
        m_worker->deleteLater();
    } else {
        delete m_worker;
    }
    m_worker = nullptr;

    if (m_workerThread != nullptr) {
        m_workerThread->quit();
        if (!m_workerThread->wait(10000)) {
            qCritical(LOG_LIVOX_MID360_SVC) << "[LivoxMid360] Worker thread did not exit in time.";
        }
    }

    delete m_workerThread;
    m_workerThread = nullptr;

    m_started = false;
    m_stopping = false;
    m_currentState = LivoxMid360RuntimeState::Stopped;
}

void LivoxMid360Service::onWorkerEnumerateFinished(
    QVector<LivoxMid360DeviceSummary> devices)
{
    emit enumerateFinished(std::move(devices));
}

void LivoxMid360Service::onWorkerOpenFinished(
    bool success,
    LivoxMid360DeviceSummary deviceInfo,
    QString errorMessage)
{
    emit openFinished(success, deviceInfo, errorMessage);
}

void LivoxMid360Service::onWorkerStateChanged(
    LivoxMid360RuntimeState newState,
    QString description)
{
    m_currentState = newState;
    emit stateChanged(newState, description);
}

void LivoxMid360Service::onWorkerLogMessage(QString message)
{
    emit logMessage(std::move(message));
}

}  // namespace livox_mid360
}  // namespace scan_tracking
