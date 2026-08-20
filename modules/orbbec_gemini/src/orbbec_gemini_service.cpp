#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/orbbec_gemini/orbbec_gemini_worker.h"

Q_LOGGING_CATEGORY(LOG_ORBBEC_GEMINI_SVC, "orbbec.gemini.service")

namespace scan_tracking {
namespace orbbec_gemini {

// 跨线程信号槽传递自定义类型前必须注册
void OrbbecGeminiService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    qRegisterMetaType<scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState>(
        "scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState");
    qRegisterMetaType<scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig>(
        "scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig");
    qRegisterMetaType<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary>(
        "scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary");
    qRegisterMetaType<scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode>(
        "scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode");
    qRegisterMetaType<scan_tracking::orbbec_gemini::OrbbecCaptureRequest>(
        "scan_tracking::orbbec_gemini::OrbbecCaptureRequest");
    qRegisterMetaType<scan_tracking::orbbec_gemini::OrbbecCaptureResult>(
        "scan_tracking::orbbec_gemini::OrbbecCaptureResult");
    qRegisterMetaType<QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary>>(
        "QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary>");
    registered = true;
}

OrbbecGeminiService::OrbbecGeminiService(QObject* parent)
    : QObject(parent)
{
}

OrbbecGeminiService::~OrbbecGeminiService()
{
    stop();
}

void OrbbecGeminiService::setOpenConfig(const OrbbecGeminiOpenConfig& config)
{
    m_openConfig = config;
    m_hasExplicitOpenConfig = true;
    m_defaultCaptureTimeoutMs =
        config.captureTimeoutMs > 0 ? config.captureTimeoutMs : 5000;
}

void OrbbecGeminiService::start()
{
    if (m_started) {
        return;
    }

    registerMetaTypes();

    // 未显式 setOpenConfig 时，从全局配置管理器加载 Orbbec 参数
    const auto* configManager = common::ConfigManager::instance();
    if (!m_hasExplicitOpenConfig && configManager != nullptr) {
        const auto& config = configManager->orbbecGeminiConfig();
        m_openConfig.serial = config.serial;
        m_openConfig.deviceIndex = config.deviceIndex;
        m_openConfig.depthWidth = config.depthWidth;
        m_openConfig.depthHeight = config.depthHeight;
        m_openConfig.fps = config.fps;
        m_openConfig.captureTimeoutMs = config.captureTimeoutMs;
        m_openConfig.warmupFrameCount = config.warmupFrameCount;
        m_openConfig.saveCaptureToDisk = config.saveCaptureToDisk;
        m_openConfig.captureCacheRoot = config.captureCacheDir;
        m_openConfig.enableColorStream = config.enableColorStream;
        m_defaultCaptureTimeoutMs =
            config.captureTimeoutMs > 0 ? config.captureTimeoutMs : 5000;
    }

    // Worker 运行在独立线程，所有 SDK 调用与采集均在子线程完成
    m_workerThread = new QThread();
    m_worker = new OrbbecGeminiWorker();
    m_worker->moveToThread(m_workerThread);

    // 主线程 → Worker：QueuedConnection 异步投递
    connect(this, &OrbbecGeminiService::sig_startWorker,
            m_worker, &OrbbecGeminiWorker::startWorker, Qt::QueuedConnection);
    connect(this, &OrbbecGeminiService::sig_stopWorker,
            m_worker, &OrbbecGeminiWorker::stopWorker, Qt::QueuedConnection);
    connect(this, &OrbbecGeminiService::sig_performCapture,
            m_worker, &OrbbecGeminiWorker::performCapture, Qt::QueuedConnection);

    // Worker → 主线程：结果信号同样走队列连接
    connect(m_worker, &OrbbecGeminiWorker::enumerateFinished,
            this, &OrbbecGeminiService::onWorkerEnumerateFinished, Qt::QueuedConnection);
    connect(m_worker, &OrbbecGeminiWorker::openFinished,
            this, &OrbbecGeminiService::onWorkerOpenFinished, Qt::QueuedConnection);
    connect(m_worker, &OrbbecGeminiWorker::stateChanged,
            this, &OrbbecGeminiService::onWorkerStateChanged, Qt::QueuedConnection);
    connect(m_worker, &OrbbecGeminiWorker::captureFinished,
            this, &OrbbecGeminiService::onWorkerCaptureFinished, Qt::QueuedConnection);
    connect(m_worker, &OrbbecGeminiWorker::logMessage,
            this, &OrbbecGeminiService::onWorkerLogMessage, Qt::QueuedConnection);

    m_workerThread->setObjectName(QStringLiteral("OrbbecGeminiWorkerThread"));
    m_workerThread->start();

    m_started = true;
    m_stopping = false;
    m_busy = false;
    m_currentState = OrbbecGeminiRuntimeState::Idle;

    emit sig_startWorker(m_openConfig);
}

void OrbbecGeminiService::stop()
{
    if (!m_started) {
        return;
    }

    m_stopping = true;
    m_busy = false;

    // 阻塞等待 Worker 在子线程中释放 SDK 资源，再销毁线程
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
            qCritical(LOG_ORBBEC_GEMINI_SVC) << "[OrbbecGemini] Worker thread did not exit in time.";
        }
    }

    delete m_workerThread;
    m_workerThread = nullptr;

    m_started = false;
    m_stopping = false;
    m_busy = false;
    m_currentState = OrbbecGeminiRuntimeState::Stopped;
}

quint64 OrbbecGeminiService::requestCapture(int timeoutMs, bool saveToDisk)
{
    if (!m_started || m_stopping || m_worker == nullptr) {
        return 0;
    }

    // 仅 Ready 且非 busy 时接受新采集，避免并发采集导致 SDK 状态异常
    if (m_busy || m_currentState != OrbbecGeminiRuntimeState::Ready) {
        return 0;
    }

    OrbbecCaptureRequest request;
    request.requestId = m_nextRequestId++;
    request.timeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;
    request.saveToDisk = saveToDisk;

    m_busy = true;
    emit sig_performCapture(request);
    return request.requestId;
}

void OrbbecGeminiService::onWorkerEnumerateFinished(
    QVector<OrbbecGeminiDeviceSummary> devices)
{
    emit enumerateFinished(std::move(devices));
}

void OrbbecGeminiService::onWorkerOpenFinished(
    bool success,
    OrbbecGeminiDeviceSummary deviceInfo,
    QString errorMessage)
{
    if (success) {
        m_currentState = OrbbecGeminiRuntimeState::Ready;
    }
    emit openFinished(success, deviceInfo, errorMessage);
}

void OrbbecGeminiService::onWorkerStateChanged(
    OrbbecGeminiRuntimeState newState,
    QString description)
{
    m_currentState = newState;
    // Capturing 期间保持 busy；其他状态（含 Ready/Failed）均视为可接受新请求
    if (newState != OrbbecGeminiRuntimeState::Capturing) {
        m_busy = false;
    }
    emit stateChanged(newState, std::move(description));
}

void OrbbecGeminiService::onWorkerCaptureFinished(OrbbecCaptureResult result)
{
    m_busy = false;
    emit captureFinished(std::move(result));
}

void OrbbecGeminiService::onWorkerLogMessage(QString message)
{
    emit logMessage(std::move(message));
}

}  // namespace orbbec_gemini
}  // namespace scan_tracking
