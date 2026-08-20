#include "scan_tracking/mech_eye/mech_eye_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QThread>

#include <mutex>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_worker.h"

Q_LOGGING_CATEGORY(LOG_MECHEYE_SVC, "mech_eye.service")

namespace scan_tracking {
namespace mech_eye {

/* 注册跨线程传递所需的 Qt 元类型。
 * queued connection 依赖元类型系统，否则参数无法安全投递到 worker 线程。
 */
void MechEyeService::registerMetaTypes()
{
    static std::once_flag once;
    std::call_once(once, [] {
        // Only direct queued-signal argument types require runtime registration.
        // Nested fields (point clouds, textures and snapshots) are copied by the
        // top-level CaptureRequest/CaptureResult metatype and must not be
        // registered separately.
        qRegisterMetaType<scan_tracking::mech_eye::CaptureErrorCode>(
            "scan_tracking::mech_eye::CaptureErrorCode");
        qInfo(LOG_MECHEYE_SVC) << "registerMetaTypes: CaptureErrorCode OK";

        qRegisterMetaType<scan_tracking::mech_eye::CameraRuntimeState>(
            "scan_tracking::mech_eye::CameraRuntimeState");
        qInfo(LOG_MECHEYE_SVC) << "registerMetaTypes: CameraRuntimeState OK";

        qRegisterMetaType<scan_tracking::mech_eye::CaptureRequest>(
            "scan_tracking::mech_eye::CaptureRequest");
        qInfo(LOG_MECHEYE_SVC) << "registerMetaTypes: CaptureRequest OK";

        qRegisterMetaType<scan_tracking::mech_eye::CaptureResult>(
            "scan_tracking::mech_eye::CaptureResult");
        qInfo(LOG_MECHEYE_SVC) << "registerMetaTypes: CaptureResult OK";
    });
}

/* 构造函数 */
MechEyeService::MechEyeService(const QString& roleName, QObject* parent)
    : QObject(parent)
    , m_roleName(roleName.trimmed())
{
}

/* 析构函数。
 * 退出时会调用 stop()，确保 worker 线程和 SDK 资源被按顺序回收。
 */
MechEyeService::~MechEyeService()
{
    stop();
}

/* 启动 Mech-Eye 服务。
 * 该流程会读取默认配置、创建 worker 线程，并把外部请求转发到后台执行。
 */
void MechEyeService::start()
{
    const auto* configManager = common::ConfigManager::instance();
    QString defaultKey;
    if (configManager != nullptr) {
        const auto cameraConfig = configManager->cameraConfig();
        const auto visionConfig = configManager->visionConfig();
        defaultKey = visionConfig.mechEyeCameraKey.trimmed();
        if (defaultKey.isEmpty()) {
            defaultKey = cameraConfig.defaultCamera;
        }
    }
    start(defaultKey);
}

void MechEyeService::start(const QString& defaultCameraKey)
{
    if (m_started) {
        return;
    }

    qInfo(LOG_MECHEYE_SVC).noquote()
        << (m_roleName.isEmpty() ? QStringLiteral("[MechEye]") : QStringLiteral("[%1]").arg(m_roleName))
        << QStringLiteral(" start: registerMetaTypes");
    registerMetaTypes();
    qInfo(LOG_MECHEYE_SVC).noquote()
        << (m_roleName.isEmpty() ? QStringLiteral("[MechEye]") : QStringLiteral("[%1]").arg(m_roleName))
        << QStringLiteral(" start: registerMetaTypes 完成");

    const auto* configManager = common::ConfigManager::instance();
    if (configManager != nullptr) {
        const auto cameraConfig = configManager->cameraConfig();
        m_defaultCameraKey = defaultCameraKey.trimmed();
        if (m_defaultCameraKey.isEmpty()) {
            const auto visionConfig = configManager->visionConfig();
            m_defaultCameraKey = visionConfig.mechEyeCameraKey.trimmed();
            if (m_defaultCameraKey.isEmpty()) {
                m_defaultCameraKey = cameraConfig.defaultCamera;
            }
        }
        m_defaultCaptureTimeoutMs = cameraConfig.scanTimeoutMs > 0 ? cameraConfig.scanTimeoutMs : 5000;
    } else {
        m_defaultCameraKey = defaultCameraKey.trimmed();
        m_defaultCaptureTimeoutMs = 5000;
    }

    qInfo(LOG_MECHEYE_SVC).noquote()
        << (m_roleName.isEmpty() ? QStringLiteral("[MechEye]") : QStringLiteral("[%1]").arg(m_roleName))
        << QStringLiteral(" start: 创建 worker key=") << m_defaultCameraKey;
    m_workerThread = new QThread();
    m_worker = new MechEyeWorker(m_roleName);

    // SDK 对象必须停留在 worker 线程，主线程只负责投递请求和接收结果。
    m_worker->moveToThread(m_workerThread);

    connect(this, &MechEyeService::sig_startWorker,
            m_worker, &MechEyeWorker::startWorker, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_stopWorker,
            m_worker, &MechEyeWorker::stopWorker, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_refreshStatus,
            m_worker, &MechEyeWorker::refreshStatus, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_performCapture,
            m_worker, &MechEyeWorker::performCapture, Qt::QueuedConnection);

    connect(m_worker, &MechEyeWorker::captureFinished,
            this, &MechEyeService::onWorkerCaptureFinished, Qt::QueuedConnection);
    connect(m_worker, &MechEyeWorker::stateChanged,
            this, &MechEyeService::onWorkerStateChanged, Qt::QueuedConnection);
    connect(m_worker, &MechEyeWorker::fatalError,
            this, &MechEyeService::onWorkerFatalError, Qt::QueuedConnection);

    m_workerThread->setObjectName(
        m_roleName.isEmpty()
            ? QStringLiteral("MechEyeWorkerThread")
            : QStringLiteral("MechEyeWorker-%1").arg(m_roleName));
    qInfo(LOG_MECHEYE_SVC).noquote()
        << (m_roleName.isEmpty() ? QStringLiteral("[MechEye]") : QStringLiteral("[%1]").arg(m_roleName))
        << QStringLiteral(" start: 启动 worker 线程");
    m_workerThread->start();

    m_started = true;
    m_busy = false;
    m_stopping = false;
    m_currentState = CameraRuntimeState::Idle;

    // Initial SDK loading/connection is a serialized startup phase. It still
    // runs on the worker thread, but completing it before other vendor modules
    // start avoids overlapping DLL static initialization and SDK helper threads.
    const bool invoked = QMetaObject::invokeMethod(
        m_worker,
        "startWorker",
        Qt::BlockingQueuedConnection,
        Q_ARG(QString, m_defaultCameraKey));
    if (!invoked) {
        qCritical(LOG_MECHEYE_SVC).noquote()
            << (m_roleName.isEmpty() ? QStringLiteral("[MechEye]") : QStringLiteral("[%1]").arg(m_roleName))
            << QStringLiteral(" start: 无法调用 worker startWorker");
    }
    qInfo(LOG_MECHEYE_SVC).noquote()
        << (m_roleName.isEmpty() ? QStringLiteral("[MechEye]") : QStringLiteral("[%1]").arg(m_roleName))
        << QStringLiteral(" start: startWorker 已完成（串行连机）");
}

/* 停止 Mech-Eye 服务。
 * 与 Orbbec 一致：先在 worker 线程阻塞释放 SDK，再 deleteLater，最后 quit/wait。
 */
void MechEyeService::stop()
{
    if (!m_started) {
        return;
    }

    m_stopping = true;
    m_busy = false;

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
            qCritical(LOG_MECHEYE_SVC) << "Mech-Eye worker 线程未能及时退出。";
        }
    }

    delete m_workerThread;
    m_workerThread = nullptr;

    m_started = false;
    m_stopping = false;
    m_busy = false;
    m_currentState = CameraRuntimeState::Stopped;
}

/* 主动刷新相机状态。
 * 当前服务未启动、正在停止或 worker 不存在时直接忽略。
 */
void MechEyeService::requestRefreshStatus()
{
    if (!m_started || m_stopping || m_worker == nullptr) {
        return;
    }

    emit sig_refreshStatus();
}

/* 发起采集请求。
 * 这里仅做状态与参数检查，真正的采集逻辑在 worker 线程执行。
 */
quint64 MechEyeService::requestCapture(const QString& cameraKey, CaptureMode mode, int timeoutMs)
{
    if (!m_started || m_stopping || m_worker == nullptr) {
        return 0;
    }

    // 单相机单并发策略：忙碌或未就绪时直接拒绝，避免队列堆积拖慢节拍。
    if (m_busy || m_currentState != CameraRuntimeState::Ready) {
        return 0;
    }

    CaptureRequest request;
    request.requestId = m_nextRequestId++;
    // 优先使用请求参数中的相机 key，若无效则退回默认配置。
    request.cameraKey = cameraKey.trimmed().isEmpty() ? m_defaultCameraKey : cameraKey.trimmed();  

    request.mode = mode;    // 采集模式
    request.timeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;  //超时设置

    m_busy = true;
    emit sig_performCapture(request);   // 发出采集请求，worker 线程会接收并执行
    return request.requestId;
}

/* 处理 worker 返回的采集完成结果。 */
void MechEyeService::onWorkerCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    m_busy = false;
    emit captureFinished(result);
}

/* 处理 worker 返回的状态变化。 */
void MechEyeService::onWorkerStateChanged(
    scan_tracking::mech_eye::CameraRuntimeState newState,
    QString description)
{
    m_currentState = newState;
    if (newState != CameraRuntimeState::Capturing) {
        m_busy = false;
    }

    // 更新相机连接状态（后续用于设备在线状态字等场景）
    m_cameraConnected = (newState == CameraRuntimeState::Ready ||
                         newState == CameraRuntimeState::Capturing);

    emit stateChanged(newState, description);
}

/* 处理 worker 返回的致命错误。 */
void MechEyeService::onWorkerFatalError(
    scan_tracking::mech_eye::CaptureErrorCode code,
    QString message)
{
    m_busy = false;
    m_lastFatalCode = code;
    m_lastFatalMessage = message;
    emit fatalError(code, message);
}

}  // namespace mech_eye
}  // namespace scan_tracking
