#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/livox_mid360/livox_mid360_service.h"
#include "scan_tracking/tfmini_plus/tfmini_plus_service.h"
#include "scan_tracking/app/console_runtime.h"

#ifdef _WIN32
#include <windows.h>
#ifdef round
#undef round
#endif
#ifdef Round
#undef Round
#endif
#endif

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QEventLoop>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QThread>
#include <QtCore/QTimer>

#include <chrono>
#include <future>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/flow_control/station3_inspection.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/hik_mono_io.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/hmi_server/hmi_tcp_server.h"

Q_LOGGING_CATEGORY(appLog, "app")

namespace scan_tracking::app {

namespace {

constexpr int kCxpWarmupConnectWaitMs = 15000;
constexpr int kCxpWarmupDelayAfterStartMs = 3000;

#ifdef _WIN32
// Windows 控制台 Ctrl+C：在独立线程中仅排队 QCoreApplication::quit()，禁止在此调用 Qt 对象或 lambda。
BOOL WINAPI handleConsoleSignal(DWORD signal)
{
    switch (signal) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT: {
        if (QCoreApplication* app = QCoreApplication::instance()) {
            QMetaObject::invokeMethod(app, "quit", Qt::QueuedConnection);
        }
        return TRUE;
    }
    default:
        return FALSE;
    }
}
#endif

bool waitCxpConnected(
    scan_tracking::vision::HikCxpCameraService* service,
    int timeoutMs,
    const std::atomic_bool* cancel)
{
    if (service == nullptr) {
        return false;
    }
    QElapsedTimer timer;
    timer.start();
    while (!service->isConnected() && timer.elapsed() < timeoutMs) {
        if (cancel != nullptr && cancel->load(std::memory_order_acquire)) {
            return false;
        }
        // 后台线程等待：不 processEvents，避免占用/阻塞主事件循环。
        QThread::msleep(50);
    }
    return service->isConnected();
}

struct CxpWarmupCaptureWaitState {
    std::promise<scan_tracking::vision::HikPoseCaptureResult> promise;
    std::atomic_bool done{false};
    quint64 requestId = 0;
    QMetaObject::Connection connection;
};

bool captureCxpWarmupFrame(
    scan_tracking::vision::HikCxpCameraService* service,
    const QString& cameraKey,
    int timeoutMs,
    const std::atomic_bool* cancel,
    scan_tracking::vision::HikPoseCaptureResult* outResult)
{
    if (service == nullptr) {
        return false;
    }

    auto state = std::make_shared<CxpWarmupCaptureWaitState>();
    auto future = state->promise.get_future();

    // 在服务所在线程（主线程）上挂槽并发起采图，结果经信号回投后再唤醒后台等待。
    const bool posted = QMetaObject::invokeMethod(
        service,
        [service, cameraKey, timeoutMs, state]() {
            state->connection = QObject::connect(
                service,
                &scan_tracking::vision::HikCxpCameraService::monoCaptureFinished,
                service,
                [state](const scan_tracking::vision::HikPoseCaptureResult& result) {
                    if (state->requestId == 0 || result.requestId != state->requestId) {
                        return;
                    }
                    QObject::disconnect(state->connection);
                    if (!state->done.exchange(true, std::memory_order_acq_rel)) {
                        state->promise.set_value(result);
                    }
                });

            state->requestId = service->requestMonoCapture(cameraKey, timeoutMs);
            if (state->requestId == 0) {
                QObject::disconnect(state->connection);
                if (!state->done.exchange(true, std::memory_order_acq_rel)) {
                    scan_tracking::vision::HikPoseCaptureResult rejected;
                    rejected.errorMessage =
                        QStringLiteral("CXP 预热采图请求被拒绝（服务忙或未启动）");
                    state->promise.set_value(std::move(rejected));
                }
            }
        },
        Qt::QueuedConnection);

    if (!posted) {
        return false;
    }

    const int waitBudgetMs = timeoutMs + 10000;
    QElapsedTimer timer;
    timer.start();
    while (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout) {
        if (cancel != nullptr && cancel->load(std::memory_order_acquire)) {
            return false;
        }
        if (timer.elapsed() >= waitBudgetMs) {
            return false;
        }
    }

    scan_tracking::vision::HikPoseCaptureResult captured = future.get();
    if (outResult != nullptr) {
        *outResult = captured;
    }
    return captured.success() && captured.frame.isValid();
}

}  // namespace

ConsoleRuntime::ConsoleRuntime(QCoreApplication& application)
    : QObject(&application)
    , application_(application)
{
}

ConsoleRuntime::~ConsoleRuntime()
{
    if (cxpWarmupCancel_ != nullptr) {
        cxpWarmupCancel_->store(true, std::memory_order_release);
    }
    if (cxpWarmupThread_.joinable()) {
        cxpWarmupThread_.join();
    }
}

int ConsoleRuntime::run()
{
#ifdef _WIN32
    SetConsoleCtrlHandler(handleConsoleSignal, TRUE);
#endif

    QObject::connect(
        &application_,
        &QCoreApplication::aboutToQuit,
        this,
        &ConsoleRuntime::handleAboutToQuit);

    printStartupStatus();
    initModules();

    const int exitCode = application_.exec();

#ifdef _WIN32
    SetConsoleCtrlHandler(handleConsoleSignal, FALSE);
#endif

    return exitCode;
}

void ConsoleRuntime::handleAboutToQuit()
{
    printShutdownStatus();
}

void ConsoleRuntime::runDeferredStartupTasks()
{
    if (shuttingDown_) {
        return;
    }
    if (hmiTcpServer_) {
        hmiTcpServer_->bindServiceSignals();
        const auto port =
            scan_tracking::common::ConfigManager::instance()->hmiConfig().tcpPort;
        if (!hmiTcpServer_->start()) {
            qWarning(appLog) << "HMI TCP 服务器在端口" << port << "启动失败。";
        } else {
            qInfo(appLog) << "HMI TCP 服务器已在端口" << port << "启动。";
        }
    }
    if (orbbecGeminiService_) {
        orbbecGeminiService_->start();
        qInfo(appLog) << QStringLiteral("[OrbbecGemini] service started.");
    }
    if (livoxMid360Service_) {
        livoxMid360Service_->start();
        qInfo(appLog) << QStringLiteral("[LivoxMid360] service started.");
    }
    for (const auto& service : tfminiPlusServices_) {
        if (!service) {
            continue;
        }
        service->start();
        const QString label = service->deviceLabel().trimmed().isEmpty()
            ? QStringLiteral("TfminiPlus")
            : service->deviceLabel().trimmed();
        qInfo(appLog).noquote()
            << QStringLiteral("[%1] service started.").arg(label);
    }

    // 给后台异步 Open 一点时间，再串行预热 A/B，避免 path3 首点才第一次连 CXP。
    if (hikCxpCameraAService_ != nullptr && hikCxpCameraBService_ != nullptr) {
        QTimer::singleShot(
            kCxpWarmupDelayAfterStartMs,
            this,
            &ConsoleRuntime::warmupCxpStereoOnStartup);
    }
}

void ConsoleRuntime::warmupCxpStereoOnStartup()
{
    if (shuttingDown_) {
        return;
    }
    if (hikCxpCameraAService_ == nullptr || hikCxpCameraBService_ == nullptr) {
        return;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        return;
    }
    const auto& visionConfig = configManager->visionConfig();
    if (!visionConfig.hikCxpEnabled) {
        return;
    }
    if (visionConfig.hikCxpBypassOk) {
        return;
    }

    if (cxpWarmupThread_.joinable()) {
        qWarning(appLog).noquote()
            << QStringLiteral("[CXP预热] 已有后台预热在运行，跳过重复启动");
        return;
    }

    const int timeoutMs =
        visionConfig.hikCxpCaptureTimeoutMs > 0 ? visionConfig.hikCxpCaptureTimeoutMs : 5000;
    const QString cameraKeyA = visionConfig.hikCxpCameraA.cameraKey;
    const QString cameraKeyB = visionConfig.hikCxpCameraB.cameraKey;
    auto* serviceA = hikCxpCameraAService_.get();
    auto* serviceB = hikCxpCameraBService_.get();

    cxpWarmupCancel_ = std::make_shared<std::atomic_bool>(false);
    auto cancel = cxpWarmupCancel_;

    qInfo(appLog).noquote()
        << QStringLiteral("[CXP预热] 已投递后台线程（不阻塞主流程）：等待双目连接并试拍出流");

    cxpWarmupThread_ = std::thread([serviceA, serviceB, cameraKeyA, cameraKeyB, timeoutMs, cancel]() {
        if (cancel->load(std::memory_order_acquire)) {
            return;
        }

        const bool aConnected =
            waitCxpConnected(serviceA, kCxpWarmupConnectWaitMs, cancel.get());
        const bool bConnected =
            waitCxpConnected(serviceB, kCxpWarmupConnectWaitMs, cancel.get());
        if (cancel->load(std::memory_order_acquire)) {
            qInfo(appLog).noquote()
                << QStringLiteral("[CXP预热] 已取消（退出中）");
            return;
        }

        qInfo(appLog).noquote()
            << QStringLiteral("[CXP预热] 连接状态 A=%1 B=%2（未连上将在采图时重试 Open）")
                   .arg(aConnected ? QStringLiteral("已连接") : QStringLiteral("未连接"))
                   .arg(bConnected ? QStringLiteral("已连接") : QStringLiteral("未连接"));

        const QString warmupDir =
            QDir(QDir::tempPath()).filePath(QStringLiteral("scan_tracking_cxp_warmup"));
        QDir().mkpath(warmupDir);

        auto warmupOne = [&](scan_tracking::vision::HikCxpCameraService* service,
                             const QString& cameraKey,
                             const QString& tag) {
            if (cancel->load(std::memory_order_acquire)) {
                return;
            }
            scan_tracking::vision::HikPoseCaptureResult result;
            if (!captureCxpWarmupFrame(
                    service, cameraKey, timeoutMs, cancel.get(), &result)) {
                if (cancel->load(std::memory_order_acquire)) {
                    return;
                }
                qWarning(appLog).noquote()
                    << QStringLiteral("[CXP预热] %1 试拍失败：%2")
                           .arg(tag, result.errorMessage.isEmpty()
                                         ? QStringLiteral("无有效帧")
                                         : result.errorMessage);
                return;
            }

            const QString bmpPath = QDir(warmupDir).filePath(
                QStringLiteral("%1_warmup.bmp").arg(tag));
            if (scan_tracking::vision::saveHikMonoFrameToBmp(result.frame, bmpPath)) {
                const bool removed = QFile::remove(bmpPath);
                qInfo(appLog).noquote()
                    << QStringLiteral("[CXP预热] %1 出流成功 %2x%3 frameId=%4 临时BMP已%5")
                           .arg(tag)
                           .arg(result.frame.width)
                           .arg(result.frame.height)
                           .arg(result.frame.frameId)
                           .arg(removed ? QStringLiteral("删除") : QStringLiteral("删除失败"));
            } else {
                qInfo(appLog).noquote()
                    << QStringLiteral("[CXP预热] %1 出流成功 %2x%3 frameId=%4（未落盘）")
                           .arg(tag)
                           .arg(result.frame.width)
                           .arg(result.frame.height)
                           .arg(result.frame.frameId);
            }
            scan_tracking::vision::releaseHikMonoFrameBuffers(&result.frame);
        };

        // 串行：SDK 开设备有互斥，与正式段扫一致先 A 后 B。
        warmupOne(serviceA, cameraKeyA, QStringLiteral("ch250_a"));
        warmupOne(serviceB, cameraKeyB, QStringLiteral("ch250_b"));

        if (cancel->load(std::memory_order_acquire)) {
            qInfo(appLog).noquote()
                << QStringLiteral("[CXP预热] 已取消（退出中）");
            return;
        }

        qInfo(appLog).noquote()
            << QStringLiteral("[CXP预热] 完成。A连接=%1 B连接=%2")
                   .arg(serviceA != nullptr && serviceA->isConnected() ? 1 : 0)
                   .arg(serviceB != nullptr && serviceB->isConnected() ? 1 : 0);
    });
}

void ConsoleRuntime::onOrbbecOpenFinished(
    bool success,
    scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary,
    const QString& errorMessage)
{
    if (shuttingDown_) {
        return;
    }
    if (!success && !errorMessage.isEmpty()) {
        qWarning(appLog).noquote()
            << QStringLiteral("[OrbbecGemini] Open failed:")
            << errorMessage;
        return;
    }
    if (!success || !orbbecCaptureOnStart_ || !orbbecSaveCaptureToDisk_) {
        return;
    }
    if (orbbecGeminiService_ == nullptr) {
        return;
    }
    const quint64 requestId =
        orbbecGeminiService_->requestCapture(orbbecCaptureTimeoutMs_, true);
    if (requestId == 0) {
        qWarning(appLog).noquote()
            << QStringLiteral("[OrbbecGemini] Startup capture request rejected");
    } else {
        qInfo(appLog).noquote()
            << QStringLiteral("[OrbbecGemini] Startup capture requested req=")
            << requestId;
    }
}

void ConsoleRuntime::initModules()
{
    qInfo(appLog) << "正在初始化模块...";

    // SCAN_TRACKING_STARTUP_STAGE 用于分步联调：0=仅 Modbus，5=全量模块
    int startupStage = 5;
    const QByteArray startupStageEnv = qgetenv("SCAN_TRACKING_STARTUP_STAGE");
	bool startupStageOk = false;
    if (!startupStageEnv.isEmpty()) {
        startupStage = startupStageEnv.toInt(&startupStageOk);
        if (!startupStageOk) {
            startupStage = 5;
        }
    }
    qInfo(appLog) << QStringLiteral("启动阶段 =") << startupStage
                  << QStringLiteral(" (0=Modbus, 1=+MechEye, 2=+Hik, 3=+VisionPipeline, 4=+Tracking, 5=+StateMachine)");

    // S3 算法模块由后续路径配置接入；当前骨架保留预热钩子。
    if (startupStage >= 5) {
        QString algorithmPrewarmError;
        if (!scan_tracking::flow_control::prewarmActiveStation3InspectionAlgorithm(
                &algorithmPrewarmError)) {
            qCritical(appLog).noquote()
                << QStringLiteral("测量算法启动期预热失败：") << algorithmPrewarmError
                << QStringLiteral("；采集流程继续，检测时将返回失败结果而不退出IPC。");
        }
    }

    modbusService_ = std::make_unique<scan_tracking::modbus::ModbusService>();
    qInfo(appLog) << "Modbus 服务已创建。";

    // stage 0：仅 Modbus，用于 PLC 通信单独验证
    if (startupStage < 1) {
        qInfo(appLog) << "启动阶段仅到 Modbus。";
        if (!modbusService_->connectDevice()) {
            qWarning(appLog) << "Modbus 连接初始化失败。";
        }
        qInfo(appLog) << "所有模块已初始化。";
        return;
    }
    // MechEye 服务先启动，保证后续状态机和视觉集成层都能复用同一份点云采集入口。
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr) {
        const auto& profile = configManager->stationProfile();
        qInfo(appLog).noquote()
            << QStringLiteral("[Station] stationId=") << scan_tracking::common::stationIdToInt(profile.stationId)
            << QStringLiteral(" name=") << profile.stationName
            << QStringLiteral(" scanPaths=") << (profile.scanPathsConfigPath.isEmpty()
                                                    ? QStringLiteral("<fallback scan_paths_config.json>")
                                                    : profile.scanPathsConfigPath)
            << QStringLiteral(" workMode=") << scan_tracking::common::workModeIdToString(profile.defaultWorkMode);
        qInfo(appLog).noquote()
            << QStringLiteral("[Station] enableLoadGrasp=") << profile.enableLoadGrasp
            << QStringLiteral(" enableUnloadCalc=") << profile.enableUnloadCalc
            << QStringLiteral(" enablePoseCheck=") << profile.enablePoseCheck
            << QStringLiteral(" enableTelescopicScan=") << profile.enableTelescopicScan
            << QStringLiteral(" enableHoistAssist=") << profile.enableHoistAssist
            << QStringLiteral(" enableCollisionMonitor=") << profile.enableCollisionMonitor
            << QStringLiteral(" (reserved, not enforced in stage1)");
    }

    const auto visionConfigForMech = configManager != nullptr
                                         ? configManager->visionConfig()
                                         : scan_tracking::common::VisionConfig{};
    const QString telescopicMechKey = visionConfigForMech.telescopicGroup.mechEye.cameraKey;
    const QString armMechKey = visionConfigForMech.armGroup.mechEye.cameraKey;
    qInfo(appLog).noquote()
        << QStringLiteral("准备启动梅卡：伸缩杆=") << telescopicMechKey
        << QStringLiteral(" 机械臂=") << armMechKey
        << QStringLiteral(" hikCxpBypassOk=") << visionConfigForMech.hikCxpBypassOk;

    mechEyeTelescopicService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>(
        QStringLiteral("梅卡-伸缩杆"));
    mechEyeArmService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>(
        QStringLiteral("梅卡-机械臂"));
    qInfo(appLog) << QStringLiteral("梅卡服务对象已创建，开始 start…");

    const auto connectMechEyeLogs = [this](scan_tracking::mech_eye::MechEyeService* service) {
        if (service == nullptr) {
            return;
        }
        // 角色名已写入 description / message（如 [梅卡-伸缩杆]），此处不再重复加前缀
        QObject::connect(
            service,
            &scan_tracking::mech_eye::MechEyeService::stateChanged,
            this,
            [](scan_tracking::mech_eye::CameraRuntimeState state, const QString& description) {
                qInfo(appLog) << QStringLiteral("状态 =") << static_cast<int>(state) << description;
            });
        QObject::connect(
            service,
            &scan_tracking::mech_eye::MechEyeService::fatalError,
            this,
            [](scan_tracking::mech_eye::CaptureErrorCode code, const QString& message) {
                qCritical(appLog) << QStringLiteral("致命错误（进程继续）：")
                                  << static_cast<int>(code) << message;
            });
    };
    connectMechEyeLogs(mechEyeTelescopicService_.get());
    connectMechEyeLogs(mechEyeArmService_.get());

    qInfo(appLog).noquote() << QStringLiteral("调用 梅卡-伸缩杆 start…");
    mechEyeTelescopicService_->start(telescopicMechKey);
    // startWorker 已在专属 worker 线程串行完成；这里只投递一次状态/错误结果。
    QCoreApplication::processEvents(QEventLoop::AllEvents);
    qInfo(appLog).noquote()
        << QStringLiteral("梅卡-伸缩杆 start() 已返回，当前状态=")
        << static_cast<int>(mechEyeTelescopicService_->state());

    const bool skipArmMechEye =
        scan_tracking::mech_eye::isMechEyeSdkProcessIsolated() ||
        mechEyeTelescopicService_->lastFatalCode() ==
            scan_tracking::mech_eye::CaptureErrorCode::SdkNativeFault;

    if (skipArmMechEye) {
        qCritical(appLog).noquote()
            << QStringLiteral("跳过梅卡-机械臂 start：伸缩杆侧已触发 MechEye SDK 原生故障隔离，")
            << QStringLiteral("同进程继续 new/connect 易二次崩溃；请重启 scan-tracking 后再试。")
            << QStringLiteral(" lastFatal=")
            << static_cast<int>(mechEyeTelescopicService_->lastFatalCode())
            << mechEyeTelescopicService_->lastFatalMessage();
    } else {
        qInfo(appLog).noquote() << QStringLiteral("调用 梅卡-机械臂 start…");
        mechEyeArmService_->start(armMechKey);
        qInfo(appLog).noquote() << QStringLiteral("梅卡-机械臂 start() 已返回");
    }
    qInfo(appLog).noquote()
        << QStringLiteral("梅卡相机服务已启动：伸缩杆=") << telescopicMechKey
        << QStringLiteral(" 机械臂=") << armMechKey
        << QStringLiteral(" armStarted=") << !skipArmMechEye;

    if (configManager != nullptr) {
        const auto& orbbecConfig = configManager->orbbecGeminiConfig();
        if (!orbbecConfig.enabled) {
            qInfo(appLog) << QStringLiteral("[OrbbecGemini] disabled (orbbecGeminiEnabled=false)");
        } else {
            orbbecGeminiService_ = std::make_unique<scan_tracking::orbbec_gemini::OrbbecGeminiService>();
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::logMessage,
                this,
                [](const QString& message) {
                    qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::stateChanged,
                this,
                [](scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState state,
                   const QString& description) {
                    qInfo(appLog).noquote()
                        << QStringLiteral("[OrbbecGemini] state=")
                        << static_cast<int>(state)
                        << description;
                });
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::captureFinished,
                this,
                [](const scan_tracking::orbbec_gemini::OrbbecCaptureResult& result) {
                    if (result.errorCode
                        != scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode::Success) {
                        qWarning(appLog).noquote()
                            << QStringLiteral("[OrbbecGemini] Capture failed:")
                            << result.errorMessage;
                        return;
                    }
                    qInfo(appLog).noquote()
                        << QStringLiteral("[OrbbecGemini] Capture saved req=") << result.requestId
                        << QStringLiteral(" depthRaw=") << result.depthRawPngPath
                        << QStringLiteral(" depthPreview=") << result.depthPreviewPngPath
                        << QStringLiteral(" pointCloud=") << result.pointCloudPlyPath;
                });
            orbbecCaptureOnStart_ = orbbecConfig.captureOnStart;
            orbbecSaveCaptureToDisk_ = orbbecConfig.saveCaptureToDisk;
            orbbecCaptureTimeoutMs_ = orbbecConfig.captureTimeoutMs;
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::openFinished,
                this,
                &ConsoleRuntime::onOrbbecOpenFinished);
            // 延后到状态机/HMI 初始化完成后再打开设备，避免 SDK 后台线程与启动期竞态。
            qInfo(appLog) << QStringLiteral("[OrbbecGemini] service created (deferred start).");
        }

        const auto& livoxConfig = configManager->livoxMid360Config();
        if (!livoxConfig.enabled) {
            qInfo(appLog) << QStringLiteral("[LivoxMid360] disabled (livoxMid360Enabled=false)");
        } else {
            livoxMid360Service_ = std::make_unique<scan_tracking::livox_mid360::LivoxMid360Service>();
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::logMessage,
                [](const QString& message) {
                    qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::stateChanged,
                [](scan_tracking::livox_mid360::LivoxMid360RuntimeState state,
                   const QString& description) {
                    qInfo(appLog).noquote()
                        << QStringLiteral("[LivoxMid360] state=")
                        << static_cast<int>(state)
                        << description;
                });
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::openFinished,
                [](bool success,
                   scan_tracking::livox_mid360::LivoxMid360DeviceSummary,
                   const QString& errorMessage) {
                    if (!success && !errorMessage.isEmpty()) {
                        qWarning(appLog).noquote()
                            << QStringLiteral("[LivoxMid360] Open failed:")
                            << errorMessage;
                    }
                });
            qInfo(appLog) << QStringLiteral("[LivoxMid360] service created (deferred start).");
        }

        const auto& tfminiConfig = configManager->tfminiPlusConfig();
        if (!tfminiConfig.enabled) {
            qInfo(appLog) << QStringLiteral("[TfminiPlus] disabled (tfminiPlusEnabled=false)");
        } else {
            // 吊装/内壁防碰辅助测距；本阶段打印测距，不写 PLC 安全位。
            auto addTfminiService = [this](
                const QString& portName,
                const QString& deviceLabel,
                int baudRate,
                bool logFrames) {
                if (portName.trimmed().isEmpty()) {
                    return;
                }

                scan_tracking::tfmini_plus::TfminiPlusOpenConfig openConfig;
                openConfig.portName = portName.trimmed();
                openConfig.baudRate = baudRate;
                openConfig.logFrames = logFrames;
                openConfig.deviceLabel = deviceLabel;

                auto service =
                    std::make_unique<scan_tracking::tfmini_plus::TfminiPlusService>();
                service->configure(openConfig);

                const QString label = deviceLabel;
                QObject::connect(
                    service.get(),
                    &scan_tracking::tfmini_plus::TfminiPlusService::logMessage,
                    [](const QString& message) {
                        qInfo(appLog).noquote() << message;
                    });
                QObject::connect(
                    service.get(),
                    &scan_tracking::tfmini_plus::TfminiPlusService::stateChanged,
                    [label](scan_tracking::tfmini_plus::TfminiPlusRuntimeState state,
                            const QString& description) {
                        qInfo(appLog).noquote()
                            << QStringLiteral("[%1] state=").arg(label)
                            << static_cast<int>(state)
                            << description;
                    });
                QObject::connect(
                    service.get(),
                    &scan_tracking::tfmini_plus::TfminiPlusService::openFinished,
                    [label](bool success, const QString& errorMessage) {
                        if (!success && !errorMessage.isEmpty()) {
                            qWarning(appLog).noquote()
                                << QStringLiteral("[%1] Open failed:").arg(label)
                                << errorMessage;
                        }
                    });

                qInfo(appLog).noquote()
                    << QStringLiteral("[%1] service created port=%2 (deferred start).")
                           .arg(label, openConfig.portName);
                tfminiPlusServices_.push_back(std::move(service));
            };

            addTfminiService(
                tfminiConfig.portName,
                QStringLiteral("TF1"),
                tfminiConfig.baudRate,
                tfminiConfig.logFrames);
            addTfminiService(
                tfminiConfig.portName2,
                QStringLiteral("TF2"),
                tfminiConfig.baudRate,
                tfminiConfig.logFrames);

            if (tfminiPlusServices_.empty()) {
                qWarning(appLog) << QStringLiteral(
                    "[TfminiPlus] enabled but both ports empty; set tfminiPlusPort / tfminiPlusPort2");
            }
        }
    }

    const auto visionConfig = configManager != nullptr
        ? configManager->visionConfig()
        : scan_tracking::common::VisionConfig{};

    if (!visionConfig.hikCxpEnabled) {
        qInfo(appLog) << QStringLiteral("CXP 双目已跳过（hikCxpEnabled=false），组合采集使用梅卡 + 海康智能 C。");
    } else if (visionConfig.hikCxpBypassOk) {
        qInfo(appLog) << QStringLiteral(
            "CXP 跳过采图（hikCxpBypassOk=true）：不启动采集卡；段扫/自检仍正常进入，CXP 不参与成败判定。");
    } else {
        // 先构造实例并接线；真正 start()/EnumDevices 延后到 StateMachine 启动之后，
        // 避免 CXP SDK 后台枚举与状态机启动并发导致进程级闪退。
        hikCxpCameraAService_ = std::make_unique<scan_tracking::vision::HikCxpCameraService>(
            QStringLiteral("ch250_a"));
        hikCxpCameraBService_ = std::make_unique<scan_tracking::vision::HikCxpCameraService>(
            QStringLiteral("ch250_b"));

        QObject::connect(
            hikCxpCameraAService_.get(),
            &scan_tracking::vision::HikCxpCameraService::stateChanged,
            [](const QString& roleName, const QString& stateText, const QString& description) {
                qInfo(appLog) << QStringLiteral("[CXP]") << roleName << stateText << description;
            });
        QObject::connect(
            hikCxpCameraBService_.get(),
            &scan_tracking::vision::HikCxpCameraService::stateChanged,
            [](const QString& roleName, const QString& stateText, const QString& description) {
                qInfo(appLog) << QStringLiteral("[CXP]") << roleName << stateText << description;
            });
        qInfo(appLog) << QStringLiteral("CXP 双目服务已创建（连接将延后启动）。");
    }

    // 海康相机 C（智能相机）：纯 TCP 控制，不通过 MVS SDK 打开设备，避免与 SCMVS 冲突
    hikCameraCController_ = std::make_unique<scan_tracking::vision::HikCameraCController>();

    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::stateChanged,
        this,
        [](scan_tracking::vision::HikCameraCState state, QString description) {
            qInfo(appLog) << QStringLiteral("[海康C控制器] 状态 =") << static_cast<int>(state) << description;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::fatalError,
        this,
        [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
            qCritical(appLog) << QStringLiteral("[海康C控制器] 致命错误：")
                              << static_cast<int>(code) << message;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::captureCompleted,
        this,
        [](scan_tracking::vision::CaptureType type,
           const QString& cameraIp,
           const QByteArray& imageData) {
            QString typeStr;
            switch (type) {
                case scan_tracking::vision::CaptureType::SurfaceDefect:
                    typeStr = QStringLiteral("表面缺陷");
                    break;
                case scan_tracking::vision::CaptureType::WeldDefect:
                    typeStr = QStringLiteral("焊缝缺陷");
                    break;
                case scan_tracking::vision::CaptureType::NumberRecognition:
                    typeStr = QStringLiteral("编号识别");
                    break;
                default:
                    typeStr = QStringLiteral("未知");
                    break;
            }
            qInfo(appLog).noquote()
                << QStringLiteral("[海康C控制器] 采集完成：") << cameraIp << typeStr
                << imageData.size() << QStringLiteral("字节");
        });

    hikCameraCController_->start(visionConfig);
    qInfo(appLog).noquote()
        << QStringLiteral("海康 C 相机控制器已启动（TCP 通信模式）：伸缩杆=")
        << visionConfig.telescopicGroup.hikCameraC.ipAddress
        << QStringLiteral(" 机械臂=")
        << visionConfig.armGroup.hikCameraC.ipAddress;


    // 统一视觉编排层负责把“1 份点云 + 2 份矩阵”收口为一个算法输入包。
    visionPipelineService_ = std::make_unique<scan_tracking::vision::VisionPipelineService>(
        mechEyeTelescopicService_.get(),
        mechEyeArmService_.get(),
        hikCxpCameraAService_.get(),
        hikCxpCameraBService_.get(),
        hikCameraCController_.get());

    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::stateChanged,
        this,
        [](scan_tracking::vision::VisionPipelineState state, const QString& description) {
            qInfo(appLog) << QStringLiteral("[视觉流水线] 状态 =") << static_cast<int>(state) << description;
        });
    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::bundleCaptureFinished,
        this,
        [](const scan_tracking::vision::MultiCameraCaptureBundle& bundle) {
            qInfo(appLog) << QStringLiteral("[视觉流水线]") << bundle.summary();
        });

    visionPipelineService_->start(visionConfig);
    qInfo(appLog) << QStringLiteral("视觉集成框架已启动。");

    // StateMachine 是主流程编排核心，注入 Modbus / 视觉等依赖
    stateMachine_ = std::make_unique<scan_tracking::flow_control::StateMachine>(
        modbusService_.get(),
        mechEyeTelescopicService_.get(),
        mechEyeArmService_.get(),
        visionPipelineService_.get(),
        hikCameraCController_.get());

    // HMI：先注入依赖；bind/listen 延后到事件循环，避免与 MechEye/视觉 worker 启动期竞态崩溃。
    const auto& hmiConfig = scan_tracking::common::ConfigManager::instance()->hmiConfig();
    if (hmiConfig.enabled) {
        hmiTcpServer_ = std::make_unique<scan_tracking::hmi_server::HmiTcpServer>(
            static_cast<int>(hmiConfig.tcpPort));
        hmiTcpServer_->setStateMachine(stateMachine_.get());
        hmiTcpServer_->setModbusService(modbusService_.get());
        hmiTcpServer_->setMechEyeServices(
            mechEyeTelescopicService_.get(), mechEyeArmService_.get());
        hmiTcpServer_->setVisionPipelineService(visionPipelineService_.get());
        hmiTcpServer_->setHikCameraServices(
            hikCxpCameraAService_.get(), hikCxpCameraBService_.get(), nullptr);
        hmiTcpServer_->setHikCameraCController(hikCameraCController_.get());
    } else {
        qInfo(appLog) << "HMI TCP 服务已在 config.ini [Hmi] enabled=false 下禁用。";
    }

    stateMachine_->start();
    qInfo(appLog) << QStringLiteral("状态机已启动。");

    // StateMachine 启动完成后再拉起 CXP SDK 枚举/连接，降低启动期进程闪退概率。
    if (visionConfig.hikCxpEnabled && hikCxpCameraAService_ && hikCxpCameraBService_) {
        hikCxpCameraAService_->start(
            visionConfig.hikCxpCameraA,
            visionConfig.hikCxpCaptureTimeoutMs,
            visionConfig.hikCxpExposureTimeUs,
            visionConfig.hikCxpGain);
        hikCxpCameraBService_->start(
            visionConfig.hikCxpCameraB,
            visionConfig.hikCxpCaptureTimeoutMs,
            visionConfig.hikCxpExposureTimeUs,
            visionConfig.hikCxpGain);
        qInfo(appLog) << QStringLiteral("CXP 双目相机服务已启动。");
    }

    // StateMachine 心跳门控会在扫描相机齐套后才启动 Modbus 监听；此处不再抢先 listen。
    qInfo(appLog) << QStringLiteral("所有模块已初始化（Modbus 将在 Mech+海康智能齐套后监听，忽略CXP）。");

    QTimer::singleShot(0, this, &ConsoleRuntime::runDeferredStartupTasks);
}

void ConsoleRuntime::printStartupStatus()
{
    qInfo(appLog).noquote()
        << "正在启动"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::name())
        << "版本"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::version());
    qInfo(appLog).noquote() << "事件循环已激活。按 Ctrl+C 退出。";
}

void ConsoleRuntime::printShutdownStatus()
{
    if (shuttingDown_) {
        return;
    }
    shuttingDown_ = true;

    // 先停后台 CXP 预热，再拆服务，避免 stop/析构时预热线程仍持有裸指针。
    if (cxpWarmupCancel_ != nullptr) {
        cxpWarmupCancel_->store(true, std::memory_order_release);
    }
    if (cxpWarmupThread_.joinable()) {
        cxpWarmupThread_.join();
    }

    // 断开本对象上的所有信号连接，避免模块 stop/析构期间再触发带捕获的槽。
    disconnect(this, nullptr, nullptr, nullptr);

    // 关闭顺序按依赖逆序执行，避免退出过程中还有异步请求落到已析构对象上。
    // HmiTcpServer 必须最先停止：它持有所有其他服务的裸指针。
    if (hmiTcpServer_) {
        hmiTcpServer_->stop();
        hmiTcpServer_.reset();
    }

    if (visionPipelineService_) {
        visionPipelineService_->stop();
    }

    if (modbusService_ && modbusService_->isConnected()) {
        modbusService_->resetIpcResultBlock();
    }

    if (stateMachine_) {
        stateMachine_->stop();
        stateMachine_.reset();
    }

    if (hikCameraCController_) {
        hikCameraCController_->stop();
        hikCameraCController_.reset();
    }
    if (visionPipelineService_) {
        visionPipelineService_.reset();
    }
    if (hikCxpCameraAService_) {
        hikCxpCameraAService_->stop();
        hikCxpCameraAService_.reset();
    }
    if (hikCxpCameraBService_) {
        hikCxpCameraBService_->stop();
        hikCxpCameraBService_.reset();
    }
    if (orbbecGeminiService_) {
        orbbecGeminiService_->stop();
        orbbecGeminiService_.reset();
    }
    if (livoxMid360Service_) {
        livoxMid360Service_->stop();
        livoxMid360Service_.reset();
    }
    for (auto& service : tfminiPlusServices_) {
        if (service) {
            service->stop();
            service.reset();
        }
    }
    tfminiPlusServices_.clear();
    if (mechEyeTelescopicService_) {
        mechEyeTelescopicService_->stop();
        mechEyeTelescopicService_.reset();
    }
    if (mechEyeArmService_) {
        mechEyeArmService_->stop();
        mechEyeArmService_.reset();
    }
    if (modbusService_) {
        modbusService_->disconnectDevice();
        modbusService_.reset();
    }

    qInfo(appLog).noquote() << "正在停止扫描跟踪核心框架。";
}

}  // namespace scan_tracking::app
