#include "scan_tracking/tfmini_plus/tfmini_plus_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tfmini_plus/tfmini_plus_worker.h"

Q_LOGGING_CATEGORY(LOG_TFMINI_PLUS_SVC, "tfmini.plus.service")

namespace scan_tracking {
namespace tfmini_plus {

void TfminiPlusService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    qRegisterMetaType<scan_tracking::tfmini_plus::TfminiPlusRuntimeState>(
        "scan_tracking::tfmini_plus::TfminiPlusRuntimeState");
    qRegisterMetaType<scan_tracking::tfmini_plus::TfminiPlusOpenConfig>(
        "scan_tracking::tfmini_plus::TfminiPlusOpenConfig");
    registered = true;
}

TfminiPlusService::TfminiPlusService(QObject* parent)
    : QObject(parent)
{
}

TfminiPlusService::~TfminiPlusService()
{
    stop();
}

void TfminiPlusService::configure(const TfminiPlusOpenConfig& config)
{
    m_openConfig = config;
    m_openConfigSet = true;
}

void TfminiPlusService::start()
{
    if (m_started) {
        return;
    }

    registerMetaTypes();

    if (!m_openConfigSet) {
        const auto* configManager = common::ConfigManager::instance();
        if (configManager != nullptr) {
            const auto& config = configManager->tfminiPlusConfig();
            m_openConfig.portName = config.portName;
            m_openConfig.baudRate = config.baudRate;
            m_openConfig.logFrames = config.logFrames;
            m_openConfig.deviceLabel = QStringLiteral("TF1");
        }
    }

    m_workerThread = new QThread();
    m_worker = new TfminiPlusWorker();
    m_worker->moveToThread(m_workerThread);

    connect(this, &TfminiPlusService::sig_startWorker,
            m_worker, &TfminiPlusWorker::startWorker, Qt::QueuedConnection);
    connect(this, &TfminiPlusService::sig_stopWorker,
            m_worker, &TfminiPlusWorker::stopWorker, Qt::QueuedConnection);

    connect(m_worker, &TfminiPlusWorker::openFinished,
            this, &TfminiPlusService::onWorkerOpenFinished, Qt::QueuedConnection);
    connect(m_worker, &TfminiPlusWorker::stateChanged,
            this, &TfminiPlusService::onWorkerStateChanged, Qt::QueuedConnection);
    connect(m_worker, &TfminiPlusWorker::distanceUpdated,
            this, &TfminiPlusService::onWorkerDistanceUpdated, Qt::QueuedConnection);
    connect(m_worker, &TfminiPlusWorker::logMessage,
            this, &TfminiPlusService::onWorkerLogMessage, Qt::QueuedConnection);

    const QString threadName = m_openConfig.deviceLabel.trimmed().isEmpty()
        ? QStringLiteral("TfminiPlusWorkerThread")
        : QStringLiteral("%1WorkerThread").arg(m_openConfig.deviceLabel.trimmed());
    m_workerThread->setObjectName(threadName);
    m_workerThread->start();

    m_started = true;
    m_currentState = TfminiPlusRuntimeState::Idle;

    emit sig_startWorker(m_openConfig);
}

void TfminiPlusService::stop()
{
    if (!m_started) {
        return;
    }

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
        if (!m_workerThread->wait(3000)) {
            qCritical(LOG_TFMINI_PLUS_SVC) << "[TfminiPlus] Worker thread did not exit in time.";
        }
    }

    delete m_workerThread;
    m_workerThread = nullptr;

    m_started = false;
    m_currentState = TfminiPlusRuntimeState::Stopped;
}

void TfminiPlusService::onWorkerOpenFinished(bool success, QString errorMessage)
{
    emit openFinished(success, std::move(errorMessage));
}

void TfminiPlusService::onWorkerStateChanged(
    TfminiPlusRuntimeState newState,
    QString description)
{
    m_currentState = newState;
    emit stateChanged(newState, std::move(description));
}

void TfminiPlusService::onWorkerDistanceUpdated(int distanceCm, int strength)
{
    emit distanceUpdated(distanceCm, strength);
}

void TfminiPlusService::onWorkerLogMessage(QString message)
{
    emit logMessage(std::move(message));
}

}  // namespace tfmini_plus
}  // namespace scan_tracking
