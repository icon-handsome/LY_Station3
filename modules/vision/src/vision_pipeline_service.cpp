#include "scan_tracking/vision/vision_pipeline_service.h"

#include <array>
#include <exception>
#include <new>
#include <thread>
#include <utility>

#include <QtCore/QLoggingCategory>
#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QMetaObject>
#include <QtCore/QMetaType>
#include <QtCore/QPointer>
#include <QtCore/QTimer>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/lb_pose_detection_adapter.h"

namespace scan_tracking {
namespace vision {

namespace {

Q_LOGGING_CATEGORY(LOG_VISION_PIPELINE, "vision.pipeline")

constexpr int kMechToHikCaptureDelayMs = 1500;

std::array<float, 16> identityMatrix4x4()
{
    return {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
}

/// LB 成功时在 worker 线程完成点云变换，避免主线程 DirectConnection 路径做百万点 Eigen。
/// 保留 pointCloudRaw=原始云，pointCloud=变换后云。
void applyLbPoseStitchingIfNeeded(MultiCameraCaptureBundle* bundle)
{
    if (bundle == nullptr) {
        return;
    }

    const auto& lb = bundle->lbPoseResult;
    if (!lb.invoked) {
        return;
    }
    if (!lb.success || !lb.poseMatrix.valid) {
        qWarning(LOG_VISION_PIPELINE).noquote()
            << QStringLiteral("LB 位姿失败，跳过点云变换：") << lb.message;
        return;
    }
    if (!bundle->mechEyeResult.pointCloud.isValid()) {
        qWarning(LOG_VISION_PIPELINE).noquote()
            << QStringLiteral("LB 成功但 Mech 点云无效，跳过变换。");
        return;
    }

    bundle->mechEyeResult.pointCloudRaw = bundle->mechEyeResult.pointCloud;

    mech_eye::PointCloudFrame stitched;
    QString stitchMessage;
    if (!mech_eye::transformPointCloudFrame(
            bundle->mechEyeResult.pointCloudRaw,
            lb.poseMatrix.values,
            identityMatrix4x4(),
            &stitched,
            &stitchMessage)) {
        bundle->mechEyeResult.pointCloudRaw = mech_eye::PointCloudFrame{};
        qWarning(LOG_VISION_PIPELINE).noquote()
            << QStringLiteral("点云 LB 变换失败：") << stitchMessage;
        return;
    }

    if (!stitched.isValid() || stitched.pointCount <= 0 ||
        stitched.pointsXYZ == nullptr ||
        static_cast<int>(stitched.pointsXYZ->size()) < stitched.pointCount * 3) {
        bundle->mechEyeResult.pointCloudRaw = mech_eye::PointCloudFrame{};
        qWarning(LOG_VISION_PIPELINE).noquote()
            << QStringLiteral("点云 LB 变换结果无效，保留原始云，不替换 pointCloud。");
        return;
    }

    bundle->mechEyeResult.pointCloud = std::move(stitched);
    qInfo(LOG_VISION_PIPELINE).noquote()
        << QStringLiteral("点云已按 LB Rt_global 变换：") << stitchMessage
        << QStringLiteral(" framePoints=") << lb.framePointCount
        << QStringLiteral(" rawKept=") << bundle->mechEyeResult.pointCloudRaw.isValid();
}

QString captureTypeLabel(CaptureType type)
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

}  // namespace

void VisionPipelineService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    qRegisterMetaType<scan_tracking::vision::VisionPipelineState>(
        "scan_tracking::vision::VisionPipelineState");
    qRegisterMetaType<scan_tracking::vision::MultiCameraCaptureBundle>(
        "scan_tracking::vision::MultiCameraCaptureBundle");
    qRegisterMetaType<scan_tracking::vision::CaptureType>("scan_tracking::vision::CaptureType");
    qRegisterMetaType<scan_tracking::vision::VisionErrorCode>(
        "scan_tracking::vision::VisionErrorCode");
    qRegisterMetaType<scan_tracking::vision::HikPoseCaptureResult>(
        "scan_tracking::vision::HikPoseCaptureResult");
    registered = true;
}

VisionPipelineService::VisionPipelineService(
    scan_tracking::mech_eye::MechEyeService* mechEyeTelescopicService,
    scan_tracking::mech_eye::MechEyeService* mechEyeArmService,
    HikCxpCameraService* hikCameraAService,
    HikCxpCameraService* hikCameraBService,
    HikCameraCController* hikCameraCController,
    QObject* parent)
    : QObject(parent)
    , m_mechEyeTelescopicService(mechEyeTelescopicService)
    , m_mechEyeArmService(mechEyeArmService)
    , m_hikCameraAService(hikCameraAService)
    , m_hikCameraBService(hikCameraBService)
    , m_hikCameraCController(hikCameraCController)
    , m_acceptLbResults(std::make_shared<std::atomic_bool>(false))
{
    registerMetaTypes();

    const auto connectMechEye = [this](scan_tracking::mech_eye::MechEyeService* service) {
        if (service == nullptr) {
            return;
        }
        connect(
            service,
            &scan_tracking::mech_eye::MechEyeService::captureFinished,
            this,
            &VisionPipelineService::onMechEyeCaptureFinished,
            Qt::QueuedConnection);
    };
    connectMechEye(m_mechEyeTelescopicService);
    connectMechEye(m_mechEyeArmService);

    if (m_hikCameraAService != nullptr) {
        connect(
            m_hikCameraAService,
            &HikCxpCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
    if (m_hikCameraBService != nullptr) {
        connect(
            m_hikCameraBService,
            &HikCxpCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
    if (m_hikCameraCController != nullptr) {
        connect(
            m_hikCameraCController,
            &HikCameraCController::imageReceived,
            this,
            &VisionPipelineService::onHikCameraCImageReceived,
            Qt::QueuedConnection);
        connect(
            m_hikCameraCController,
            &HikCameraCController::captureCompleted,
            this,
            &VisionPipelineService::onHikCameraCCaptureCompleted,
            Qt::QueuedConnection);
    }
}

VisionPipelineService::~VisionPipelineService()
{
    stop();
}

void VisionPipelineService::joinLbPoseThread()
{
    {
        std::lock_guard<std::mutex> lock(m_lbPoseThreadMutex);
        m_lbPoseWorkerStopping = true;
    }
    m_lbPoseQueueCv.notify_all();

    std::thread lbThread;
    {
        std::lock_guard<std::mutex> lock(m_lbPoseThreadMutex);
        lbThread = std::move(m_lbPoseThread);
    }
    if (lbThread.joinable()) {
        lbThread.join();
    }

    std::lock_guard<std::mutex> lock(m_lbPoseThreadMutex);
    m_lbPoseQueue.clear();
    m_lbPoseWorkerRunning = false;
    m_lbPoseWorkerStopping = false;
}

void VisionPipelineService::ensureLbPoseWorkerRunning()
{
    if (m_lbPoseWorkerRunning) {
        return;
    }
    m_lbPoseThread = std::thread([this]() { lbPoseWorkerLoop(); });
    m_lbPoseWorkerRunning = true;
}

void VisionPipelineService::enqueueLbPoseJob(MultiCameraCaptureBundle bundle)
{
    const auto acceptResults = m_acceptLbResults;

    {
        std::lock_guard<std::mutex> lock(m_lbPoseThreadMutex);
        if (m_lbPoseWorkerStopping ||
            !m_started ||
            acceptResults == nullptr ||
            !acceptResults->load(std::memory_order_acquire)) {
            bundle.lbPoseResult.invoked = false;
            bundle.lbPoseResult.success = false;
            bundle.lbPoseResult.message = QStringLiteral("流水线已停止，跳过 LB 位姿检测。");
            emitBundleFinished(std::move(bundle));
            return;
        }

        LbPoseJob job;
        job.bundle = std::move(bundle);
        job.lbConfig = m_lbPoseConfig;
        m_lbPoseQueue.push_back(std::move(job));
        ensureLbPoseWorkerRunning();
    }
    m_lbPoseQueueCv.notify_one();
}

void VisionPipelineService::lbPoseWorkerLoop()
{
    const auto acceptResults = m_acceptLbResults;
    VisionPipelineService* const receiver = this;

    for (;;) {
        LbPoseJob job;
        {
            std::unique_lock<std::mutex> lock(m_lbPoseThreadMutex);
            m_lbPoseQueueCv.wait(lock, [this]() {
                return m_lbPoseWorkerStopping || !m_lbPoseQueue.empty();
            });
            if (m_lbPoseWorkerStopping && m_lbPoseQueue.empty()) {
                break;
            }
            job = std::move(m_lbPoseQueue.front());
            m_lbPoseQueue.pop_front();
        }

        // 现场立体约定：CXP-A = 右目（I2），CXP-B = 左目（I1）。
        job.bundle.lbPoseResult = runLbPoseDetection(
            job.bundle.hikCameraBResult.frame,
            job.bundle.hikCameraAResult.frame,
            job.lbConfig);

        // 拼接放在 LB worker，主线程只做缓存/ACK/投递落盘。
        // 必须吞掉异常：std::thread 未捕获 → terminate 闪退。
        try {
            applyLbPoseStitchingIfNeeded(&job.bundle);
        } catch (const std::bad_alloc&) {
            qCritical(LOG_VISION_PIPELINE).noquote()
                << QStringLiteral("LB 点云拼接内存不足，保留原始云继续交付。");
            job.bundle.mechEyeResult.pointCloudRaw = mech_eye::PointCloudFrame{};
        } catch (const std::exception& ex) {
            qCritical(LOG_VISION_PIPELINE).noquote()
                << QStringLiteral("LB 点云拼接异常，保留原始云：")
                << QString::fromUtf8(ex.what());
            job.bundle.mechEyeResult.pointCloudRaw = mech_eye::PointCloudFrame{};
        } catch (...) {
            qCritical(LOG_VISION_PIPELINE).noquote()
                << QStringLiteral("LB 点云拼接未知异常，保留原始云继续交付。");
            job.bundle.mechEyeResult.pointCloudRaw = mech_eye::PointCloudFrame{};
        }

        if (!acceptResults->load(std::memory_order_acquire)) {
            continue;
        }

        QMetaObject::invokeMethod(
            receiver,
            [acceptResults, receiver, completed = std::move(job.bundle)]() mutable {
                if (!acceptResults->load(std::memory_order_acquire)) {
                    return;
                }
                receiver->emitBundleFinished(std::move(completed));
            },
            Qt::QueuedConnection);
    }

    std::lock_guard<std::mutex> lock(m_lbPoseThreadMutex);
    m_lbPoseWorkerRunning = false;
}

void VisionPipelineService::start(const scan_tracking::common::VisionConfig& config)
{
    // 若上次 stop 后仍有未接合线程，先收口再开闸。
    joinLbPoseThread();
    m_config = config;
    if (const auto* cfg = scan_tracking::common::ConfigManager::instance()) {
        m_lbPoseConfig = cfg->lbPoseConfig();
    }
    m_pending = PendingCaptureContext{};
    m_processing = false;
    m_acceptLbResults->store(true, std::memory_order_release);
    m_started = true;
    setState(
        VisionPipelineState::Ready,
        QStringLiteral("视觉流水线已启动，等待采集请求。"));
}

void VisionPipelineService::stop()
{
    m_acceptLbResults->store(false, std::memory_order_release);
    m_pending = PendingCaptureContext{};
    const bool wasStarted = m_started;
    m_started = false;
    joinLbPoseThread();
    m_processing = false;
    if (wasStarted) {
        setState(VisionPipelineState::Stopped, QStringLiteral("视觉流水线已停止。"));
    }
}

bool VisionPipelineService::isHikCxpAConnected() const
{
    return m_hikCameraAService != nullptr && m_hikCameraAService->isConnected();
}

bool VisionPipelineService::isHikCxpBConnected() const
{
    return m_hikCameraBService != nullptr && m_hikCameraBService->isConnected();
}

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode)
{
    return requestCaptureBundle(segmentIndex, taskId, mechCaptureMode, false);
}

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode,
    bool telescopicConcurrentHikC)
{
    // HMI / 旧调用：保持「全局 CXP 开关 + 控制器存在则采智能 C」的旧行为。
    BundleCaptureOptions options;
    options.useMechEye = true;
    options.useHikCxp = true;
    options.useHikSmartC = true;
    return requestCaptureBundle(
        segmentIndex, taskId, mechCaptureMode, telescopicConcurrentHikC, options);
}

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode,
    bool telescopicConcurrentHikC,
    BundleCaptureOptions options)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("视觉流水线未启动。"));
        return 0;
    }
    if (m_pending.active || m_processing) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("视觉采集请求正在进行中。"));
        return 0;
    }

    const scan_tracking::common::VisionDeviceGroupConfig& deviceGroup =
        telescopicConcurrentHikC ? m_config.telescopicGroup : m_config.armGroup;
    scan_tracking::mech_eye::MechEyeService* mechService =
        telescopicConcurrentHikC ? m_mechEyeTelescopicService : m_mechEyeArmService;

    if (options.useMechEye && mechService == nullptr) {
        emit fatalError(
            VisionErrorCode::InvalidConfig,
            telescopicConcurrentHikC
                ? QStringLiteral("伸缩杆梅卡相机服务不可用。")
                : QStringLiteral("机械臂梅卡相机服务不可用。"));
        return 0;
    }
    if (!options.useMechEye) {
        // 本轮段扫仍依赖 Mech；关闭 Mech 的路径（如编号）应走专用采集入口。
        emit fatalError(
            VisionErrorCode::InvalidConfig,
            QStringLiteral("组合采集当前仍需要梅卡 3D；请使用路径专用采集入口。"));
        return 0;
    }

    // 机械臂：可并行 CXP（LB）+ 海康 C；伸缩杆：仅海康 C。
    // hikCxpBypassOk：路径仍正常进（梅卡/海康C照采），仅不采 CXP、不把 CXP 计入成败、不报采图失败。
    const bool pathWantsCxp = options.useHikCxp && !telescopicConcurrentHikC;
    const bool skipCxpCapture = pathWantsCxp && m_config.hikCxpBypassOk;
    const bool useCxp =
        pathWantsCxp && !skipCxpCapture && m_config.hikCxpEnabled &&
        m_hikCameraAService != nullptr && m_hikCameraBService != nullptr;
    const bool useHikCameraC =
        options.useHikSmartC && m_hikCameraCController != nullptr;

    MultiCameraCaptureRequest request;
    request.requestId = m_nextRequestId++;
    request.taskId = taskId;
    request.segmentIndex = segmentIndex;
    request.mechCaptureMode = mechCaptureMode;
    request.needMechEye2D =
        mechCaptureMode == scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D;
    request.mechEyeCameraKey = deviceGroup.mechEye.cameraKey;
    request.mechEyeTimeoutMs =
        m_config.mechCaptureTimeoutMs > 0 ? m_config.mechCaptureTimeoutMs : 5000;
    if (useHikCameraC) {
        request.hikCameraCIp = deviceGroup.hikCameraC.ipAddress;
    }
    // 仅真实采图时写入 CXP key；旁路时不写入，避免 cxpParticipated() 把空结果判成失败。
    if (useCxp) {
        request.hikCameraAKey = m_config.hikCxpCameraA.cameraKey;
        request.hikCameraBKey = m_config.hikCxpCameraB.cameraKey;
        request.hikTimeoutMs =
            m_config.hikCxpCaptureTimeoutMs > 0 ? m_config.hikCxpCaptureTimeoutMs : 5000;
    }

    PendingCaptureContext pending;
    pending.active = true;
    pending.useCxp = useCxp;
    pending.useHikCameraC = useHikCameraC;
    pending.hikCTriggerOnly = useHikCameraC;
    pending.hikCameraCIp = useHikCameraC ? deviceGroup.hikCameraC.ipAddress.trimmed() : QString();
    pending.activeMechService = mechService;
    pending.bundle.request = request;
    // 未参与的通道视为已完成，避免 finishBundleIfReady 死等。
    if (!useCxp) {
        pending.hikADone = true;
        pending.hikBDone = true;
    }
    if (skipCxpCapture) {
        pending.bundle.lbPoseResult.invoked = false;
        pending.bundle.lbPoseResult.success = false;
        pending.bundle.lbPoseResult.message =
            QStringLiteral("hikCxpBypassOk=true：本段不采 CXP，跳过 LB。");
    }
    if (!useHikCameraC) {
        pending.hikCDone = true;
    }

    pending.mechRequestId = mechService->requestCapture(
        request.mechEyeCameraKey,
        mechCaptureMode,
        request.mechEyeTimeoutMs);
    if (pending.mechRequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("启动 Mech-Eye 采集失败。"));
        return 0;
    }

    m_pending = pending;

    QStringList parts;
    parts << QStringLiteral("梅卡");
    if (useCxp) {
        parts << QStringLiteral("CXP");
    } else if (skipCxpCapture) {
        parts << QStringLiteral("CXP(跳过采图)");
    }
    if (useHikCameraC) {
        parts << QStringLiteral("海康C");
    }
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("梅卡采集已启动（%1；海康通道将在梅卡完成后延迟 %2ms）")
            .arg(parts.join(QStringLiteral("+")))
            .arg(kMechToHikCaptureDelayMs));
    qInfo(LOG_VISION_PIPELINE).noquote()
        << QStringLiteral("[VisionPipeline] 组合采集 requestId=%1 segment=%2 device=%3 "
                          "channels=%4 pathOpts(cxp=%5 smart=%6) cxpSkipCapture=%7")
               .arg(request.requestId)
               .arg(segmentIndex)
               .arg(telescopicConcurrentHikC ? QStringLiteral("telescopic")
                                             : QStringLiteral("arm"))
               .arg(parts.join(QLatin1Char('+')))
               .arg(options.useHikCxp)
               .arg(options.useHikSmartC)
               .arg(skipCxpCapture);
    return request.requestId;
}

void VisionPipelineService::startPendingHikCapture()
{
    if (!m_pending.active || !m_pending.useCxp || m_pending.hikARequestId != 0) {
        return;
    }

    const auto& request = m_pending.bundle.request;
    m_pending.hikARequestId = m_hikCameraAService->requestPoseCapture(
        request.hikCameraAKey, request.hikTimeoutMs);
    m_pending.hikBRequestId = m_hikCameraBService->requestPoseCapture(
        request.hikCameraBKey, request.hikTimeoutMs);

    if (m_pending.hikARequestId == 0) {
        m_pending.bundle.hikCameraAResult.logicalName = m_config.hikCxpCameraA.logicalName;
        m_pending.bundle.hikCameraAResult.errorCode = VisionErrorCode::CaptureRejected;
        m_pending.bundle.hikCameraAResult.errorMessage =
            QStringLiteral("CXP 右目(A)采集启动失败。");
        m_pending.hikADone = true;
    }
    if (m_pending.hikBRequestId == 0) {
        m_pending.bundle.hikCameraBResult.logicalName = m_config.hikCxpCameraB.logicalName;
        m_pending.bundle.hikCameraBResult.errorCode = VisionErrorCode::CaptureRejected;
        m_pending.bundle.hikCameraBResult.errorMessage =
            QStringLiteral("CXP 左目(B)采集启动失败。");
        m_pending.hikBDone = true;
    }
    if (m_pending.hikADone && m_pending.hikBDone) {
        finishBundleIfReady();
        return;
    }

    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("CXP 双目采集已启动：requestId=%1").arg(request.requestId));
}

void VisionPipelineService::triggerHikCameraCConcurrent(bool triggerOnly)
{
    Q_UNUSED(triggerOnly);
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone) {
        return;
    }

    const bool sent =
        m_hikCameraCController != nullptr &&
        m_hikCameraCController->requestCapture(
            CaptureType::SurfaceDefect,
            m_pending.hikCameraCIp);

    m_pending.hikCDone = true;
    m_pending.bundle.hikCameraCTriggerOk = sent;
    m_pending.bundle.hikCameraCImagePath.clear();

    const QString hikCGroup =
        (m_pending.hikCameraCIp == m_config.telescopicGroup.hikCameraC.ipAddress.trimmed())
            ? QStringLiteral("[海康C-伸缩杆]")
            : (m_pending.hikCameraCIp == m_config.armGroup.hikCameraC.ipAddress.trimmed())
                  ? QStringLiteral("[海康C-机械臂]")
                  : QStringLiteral("[海康C]");

    if (!sent) {
        qWarning(LOG_VISION_PIPELINE).noquote()
            << QStringLiteral("[VisionPipeline]") << hikCGroup
            << QStringLiteral(" start 发送失败（TCP 未连接或未就绪） IP=")
            << m_pending.hikCameraCIp;
        return;
    }

    qInfo(LOG_VISION_PIPELINE).noquote()
        << QStringLiteral("[VisionPipeline]") << hikCGroup
        << QStringLiteral(" start 已发送 requestId=")
        << m_pending.bundle.request.requestId
        << QStringLiteral(" IP=") << m_pending.hikCameraCIp;
}

void VisionPipelineService::startPendingHikCameraCCapture()
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone) {
        return;
    }

    triggerHikCameraCConcurrent(false);

    if (m_pending.hikCDone) {
        finishBundleIfReady();
        return;
    }

    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("海康 C 采集已触发：requestId=%1 IP=%2")
            .arg(m_pending.bundle.request.requestId)
            .arg(m_pending.hikCameraCIp));
}

void VisionPipelineService::completeHikCameraCCapture(const QString& imagePath)
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone ||
        m_pending.hikCTriggerOnly) {
        return;
    }

    if (imagePath.trimmed().isEmpty() || !QFile::exists(imagePath)) {
        return;
    }

    m_pending.bundle.hikCameraCImagePath = imagePath;
    m_pending.hikCDone = true;
    finishBundleIfReady();
}

void VisionPipelineService::onHikCameraCCaptureTimeout()
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone) {
        return;
    }

    m_pending.hikCDone = true;
    m_pending.bundle.hikCameraCImagePath.clear();
    finishBundleIfReady();
}

void VisionPipelineService::onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    if (!m_pending.active || result.requestId != m_pending.mechRequestId) {
        return;
    }

    const auto* senderService = qobject_cast<scan_tracking::mech_eye::MechEyeService*>(sender());
    if (senderService != nullptr && senderService != m_pending.activeMechService) {
        return;
    }

    m_pending.bundle.mechEyeResult = result;
    m_pending.mechDone = true;

    QPointer<VisionPipelineService> self(this);
    QTimer::singleShot(kMechToHikCaptureDelayMs, this, [self]() {
        if (self == nullptr || !self->m_pending.active) {
            return;
        }
        // 机械臂路径：并行启动 CXP 与海康 C。
        if (self->m_pending.useCxp) {
            self->startPendingHikCapture();
        }
        if (self->m_pending.useHikCameraC) {
            self->startPendingHikCameraCCapture();
        }
        if (!self->m_pending.useCxp && !self->m_pending.useHikCameraC) {
            self->finishBundleIfReady();
        }
    });
}

void VisionPipelineService::onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result)
{
    if (!m_pending.active || !m_pending.useCxp) {
        return;
    }

    if (result.logicalName == m_config.hikCxpCameraA.logicalName) {
        m_pending.bundle.hikCameraAResult = result;
        m_pending.hikADone = true;
    } else if (result.logicalName == m_config.hikCxpCameraB.logicalName) {
        m_pending.bundle.hikCameraBResult = result;
        m_pending.hikBDone = true;
    } else {
        return;
    }

    finishBundleIfReady();
}

void VisionPipelineService::onHikCameraCImageReceived(
    scan_tracking::vision::CaptureType type,
    QString cameraIp,
    QString filePath,
    qint64 fileSize)
{
    Q_UNUSED(type);
    Q_UNUSED(fileSize);
    if (!m_pending.active || m_pending.hikCTriggerOnly) {
        return;
    }
    if (cameraIp.trimmed() != m_pending.hikCameraCIp) {
        return;
    }
    completeHikCameraCCapture(filePath);
}

void VisionPipelineService::onHikCameraCCaptureCompleted(
    scan_tracking::vision::CaptureType type,
    QString cameraIp,
    QByteArray imageData)
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone ||
        m_pending.hikCTriggerOnly) {
        return;
    }

    if (cameraIp.trimmed() != m_pending.hikCameraCIp) {
        return;
    }

    if (imageData.isEmpty()) {
        return;
    }

    const QString saveDir = QStringLiteral("./smart_camera_images");
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString ipTag = cameraIp.trimmed().replace(QLatin1Char('.'), QLatin1Char('_'));
    const QString filePath = QStringLiteral("%1/pipeline_%2_%3_%4.jpg")
                                 .arg(saveDir, captureTypeLabel(type), ipTag, timestamp);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        return;
    }
    if (file.write(imageData) != imageData.size()) {
        return;
    }
    file.close();

    completeHikCameraCCapture(filePath);
}

void VisionPipelineService::setState(VisionPipelineState state, const QString& description)
{
    m_state = state;
    emit stateChanged(state, description);
}

void VisionPipelineService::emitBundleFinished(MultiCameraCaptureBundle bundle)
{
    m_processing = false;
    const bool ok = bundle.success();
    setState(
        ok ? VisionPipelineState::Ready : VisionPipelineState::Error,
        ok ? QStringLiteral("视觉组合采集成功完成。")
           : QStringLiteral("视觉组合采集完成但有错误。"));
    qInfo(LOG_VISION_PIPELINE).noquote() << bundle.summary()
        << QStringLiteral(" LB=") << (bundle.lbPoseResult.invoked
            ? (bundle.lbPoseResult.success ? QStringLiteral("ok") : bundle.lbPoseResult.message)
            : QStringLiteral("skip"));
    emit bundleCaptureFinished(bundle);
}

void VisionPipelineService::finishBundleIfReady()
{
    if (!m_pending.active || !m_pending.mechDone) {
        return;
    }
    if (!m_pending.hikADone || !m_pending.hikBDone || !m_pending.hikCDone) {
        return;
    }

    auto bundle = m_pending.bundle;
    const bool runLb = m_pending.useCxp;
    m_pending = PendingCaptureContext{};

    if (!runLb) {
        bundle.lbPoseResult.invoked = false;
        bundle.lbPoseResult.success = false;
        bundle.lbPoseResult.message = QStringLiteral("本段未启用 CXP，跳过 LB 位姿检测。");
        emitBundleFinished(std::move(bundle));
        return;
    }

    const bool hikReady =
        bundle.hikCameraAResult.success() && bundle.hikCameraBResult.success();
    if (!hikReady) {
        bundle.lbPoseResult.invoked = false;
        bundle.lbPoseResult.success = false;
        bundle.lbPoseResult.message = QStringLiteral("CXP 双目未就绪，跳过 LB 位姿检测。");
        emitBundleFinished(std::move(bundle));
        return;
    }

    m_processing = true;
    setState(VisionPipelineState::Capturing, QStringLiteral("正在执行 LB 位姿检测…"));
    enqueueLbPoseJob(std::move(bundle));
}

}  // namespace vision
}  // namespace scan_tracking
