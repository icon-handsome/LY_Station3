#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/hik_cxp_mono_io.h"
#include "scan_tracking/vision/hik_mono_io.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QEventLoop>
#include <QtCore/QTextStream>
#include <QtCore/QDateTime>
#include <QtCore/QThread>
#include <QtCore/QTimer>

namespace {

using scan_tracking::vision::HikCxpCameraService;
using scan_tracking::vision::HikPoseCaptureResult;

struct CaptureContext {
    HikCxpCameraService* service = nullptr;
    QEventLoop* loop = nullptr;
    bool ok = false;
    HikPoseCaptureResult result;
};

bool runSingleCapture(
    HikCxpCameraService& service,
    const QString& cameraKey,
    int timeoutMs,
    HikPoseCaptureResult* outResult,
    QTextStream& log)
{
    CaptureContext ctx;
    ctx.service = &service;
    QEventLoop loop;
    ctx.loop = &loop;

    QMetaObject::Connection conn = QObject::connect(
        &service,
        &HikCxpCameraService::monoCaptureFinished,
        &loop,
        [&ctx](const HikPoseCaptureResult& result) {
            ctx.result = result;
            ctx.ok = result.success();
            if (ctx.loop != nullptr) {
                ctx.loop->quit();
            }
        });

    QTimer watchdog;
    watchdog.setSingleShot(true);
    QObject::connect(&watchdog, &QTimer::timeout, &loop, &QEventLoop::quit);
    watchdog.start(static_cast<int>(timeoutMs + 15000));

    const quint64 requestId = service.requestMonoCapture(cameraKey, timeoutMs);
    if (requestId == 0) {
        QObject::disconnect(conn);
        log << "  [ERROR] requestMonoCapture rejected for " << cameraKey << "\n";
        return false;
    }

    loop.exec();
    QObject::disconnect(conn);

    if (!ctx.ok) {
        log << "  [ERROR] capture failed: " << ctx.result.errorMessage << "\n";
        return false;
    }

    if (outResult != nullptr) {
        *outResult = ctx.result;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTextStream log(stdout);

    scan_tracking::common::ConfigManager::initialize();
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        log << "[ERROR] ConfigManager 未初始化\n";
        log.flush();
        return 1;
    }
    const auto visionConfig = configManager->visionConfig();

    log << "\n--- [ CXP Smoke Test Started ] ---\n";
    log << "config (exe dir): "
        << QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("config.ini"))
        << "\n";

    if (!visionConfig.hikCxpEnabled) {
        log << "[ERROR] hikCxpEnabled=false in config.ini\n";
        log.flush();
        scan_tracking::common::ConfigManager::cleanup();
        return 1;
    }

    const QString outputDir = visionConfig.hikCxpSmokeOutputDir.trimmed();
    if (outputDir.isEmpty()) {
        log << "[ERROR] hikCxpSmokeOutputDir is empty\n";
        log.flush();
        scan_tracking::common::ConfigManager::cleanup();
        return 1;
    }
    if (!QDir().mkpath(outputDir)) {
        log << "[ERROR] cannot create output dir: " << outputDir << "\n";
        log.flush();
        scan_tracking::common::ConfigManager::cleanup();
        return 1;
    }
    log << "output dir: " << outputDir << "\n";

    HikCxpCameraService cameraA(QStringLiteral("ch250_a"));
    HikCxpCameraService cameraB(QStringLiteral("ch250_b"));

    cameraA.start(
        visionConfig.hikCxpCameraA,
        visionConfig.hikCxpCaptureTimeoutMs,
        visionConfig.hikCxpExposureTimeUs,
        visionConfig.hikCxpGain);
    cameraB.start(
        visionConfig.hikCxpCameraB,
        visionConfig.hikCxpCaptureTimeoutMs,
        visionConfig.hikCxpExposureTimeUs,
        visionConfig.hikCxpGain);

  // 等待后台连接尝试
    QThread::msleep(2000);

    bool allOk = true;

    log << "\n1. Capture camera A (" << visionConfig.hikCxpCameraA.cameraKey << ")...\n";
  HikPoseCaptureResult resultA;
    if (!runSingleCapture(
            cameraA,
            visionConfig.hikCxpCameraA.cameraKey,
            visionConfig.hikCxpCaptureTimeoutMs,
            &resultA,
            log)) {
        allOk = false;
    } else {
        const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
        const QString path = scan_tracking::vision::buildCxpSmokeBmpPath(
            outputDir, cameraA.roleName(), ts);
        if (!scan_tracking::vision::saveHikMonoFrameToBmp(resultA.frame, path)) {
            log << "  [ERROR] save BMP failed: " << path << "\n";
            allOk = false;
        } else {
            log << "  [OK] saved: " << path << " (" << resultA.frame.width << "x"
                << resultA.frame.height << ")\n";
        }
    }

    log << "\n2. Capture camera B (" << visionConfig.hikCxpCameraB.cameraKey << ")...\n";
    HikPoseCaptureResult resultB;
    if (!runSingleCapture(
            cameraB,
            visionConfig.hikCxpCameraB.cameraKey,
            visionConfig.hikCxpCaptureTimeoutMs,
            &resultB,
            log)) {
        allOk = false;
    } else {
        const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
        const QString path = scan_tracking::vision::buildCxpSmokeBmpPath(
            outputDir, cameraB.roleName(), ts);
        if (!scan_tracking::vision::saveHikMonoFrameToBmp(resultB.frame, path)) {
            log << "  [ERROR] save BMP failed: " << path << "\n";
            allOk = false;
        } else {
            log << "  [OK] saved: " << path << " (" << resultB.frame.width << "x"
                << resultB.frame.height << ")\n";
        }
    }

    cameraA.stop();
    cameraB.stop();

    log << "\n--- [ CXP Smoke Test " << (allOk ? "PASSED" : "FAILED") << " ] ---\n\n";
    log.flush();

    scan_tracking::common::ConfigManager::cleanup();
    return allOk ? 0 : 1;
}
