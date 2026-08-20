/**
 * @file gemini335_smoke_runner.cpp
 * @brief Orbbec Gemini 独立冒烟测试程序
 *
 * 命令行直接驱动 OrbbecGeminiWorker（不经 Service 层），用于联调设备连接与采集落盘。
 */

#include <QtCore/QCoreApplication>
#include <QtCore/QEventLoop>
#include <QtCore/QLoggingCategory>
#include <QtCore/QString>
#include <QtCore/QTextStream>
#include <QtCore/QTimer>

#include "scan_tracking/orbbec_gemini/orbbec_gemini_worker.h"

Q_LOGGING_CATEGORY(appLog, "app")

namespace {

void printUsage()
{
    qInfo(appLog).noquote()
        << "Usage: gemini335_smoke_runner [options]\n"
        << "  --serial SN        Open device by serial number\n"
        << "  --index N          Open device by index (default 0)\n"
        << "  --capture          Capture one frame after open\n"
        << "  --count N          Capture count (default 1, requires --capture)\n"
        << "  --timeout MS       Capture timeout in ms (default 5000)\n"
        << "  --cache-dir PATH   Capture cache root (default: exe/ScanTracking_CaptureCache)\n"
        << "  -h, --help         Show this help";
}

struct RunnerOptions {
    scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig openConfig;
    bool doCapture = false;
    int captureCount = 1;
    int captureTimeoutMs = 5000;
};

bool parseArgs(const QStringList& args, RunnerOptions* options)
{
    if (options == nullptr) {
        return false;
    }

    for (int index = 1; index < args.size(); ++index) {
        const QString& token = args.at(index);
        if (token == QStringLiteral("-h") || token == QStringLiteral("--help")) {
            printUsage();
            return false;
        }
        if (token == QStringLiteral("--serial") && index + 1 < args.size()) {
            options->openConfig.serial = args.at(++index);
            continue;
        }
        if (token == QStringLiteral("--index") && index + 1 < args.size()) {
            options->openConfig.deviceIndex = args.at(++index).toInt();
            continue;
        }
        if (token == QStringLiteral("--capture")) {
            options->doCapture = true;
            continue;
        }
        if (token == QStringLiteral("--count") && index + 1 < args.size()) {
            options->captureCount = args.at(++index).toInt();
            continue;
        }
        if (token == QStringLiteral("--timeout") && index + 1 < args.size()) {
            options->captureTimeoutMs = args.at(++index).toInt();
            continue;
        }
        if (token == QStringLiteral("--cache-dir") && index + 1 < args.size()) {
            options->openConfig.captureCacheRoot = args.at(++index);
            continue;
        }
    }

    options->openConfig.saveCaptureToDisk = true;
    options->openConfig.captureTimeoutMs = options->captureTimeoutMs;
    if (options->captureCount <= 0) {
        options->captureCount = 1;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("gemini335_smoke_runner"));

    RunnerOptions options;
    if (!parseArgs(QCoreApplication::arguments(), &options)) {
        return 0;
    }

    // 冒烟程序在同一线程直接调用 Worker，无需 QThread 封装
    scan_tracking::orbbec_gemini::OrbbecGeminiWorker worker;
    QObject::connect(
        &worker,
        &scan_tracking::orbbec_gemini::OrbbecGeminiWorker::logMessage,
        [](const QString& message) {
            qInfo(appLog).noquote() << message;
            QTextStream stream(stdout);
            stream << message << '\n';
            stream.flush();
        });

    int exitCode = 0;
    int remainingCaptures = options.doCapture ? options.captureCount : 0;
    QEventLoop loop;

    // 打开成功后按需触发首次采集；无 --capture 时仅验证枚举与打开
    QObject::connect(
        &worker,
        &scan_tracking::orbbec_gemini::OrbbecGeminiWorker::openFinished,
        &worker,
        [&](bool success, scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary,
            const QString& errorMessage) {
            if (!success) {
                qCritical(appLog).noquote()
                    << QStringLiteral("Open failed:") << errorMessage;
                exitCode = 1;
                loop.quit();
                return;
            }
            if (remainingCaptures <= 0) {
                loop.quit();
                return;
            }

            scan_tracking::orbbec_gemini::OrbbecCaptureRequest request;
            request.requestId = 1;
            request.timeoutMs = options.captureTimeoutMs;
            request.saveToDisk = true;
            worker.performCapture(request);
        });

    QObject::connect(
        &worker,
        &scan_tracking::orbbec_gemini::OrbbecGeminiWorker::captureFinished,
        &worker,
        [&](scan_tracking::orbbec_gemini::OrbbecCaptureResult result) {
            if (result.errorCode
                != scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode::Success) {
                qCritical(appLog).noquote()
                    << QStringLiteral("Capture failed:") << result.errorMessage;
                exitCode = 3;
                loop.quit();
                return;
            }

            qInfo(appLog).noquote()
                << QStringLiteral("Capture saved depthRaw=") << result.depthRawPngPath
                << QStringLiteral(" depthPreview=") << result.depthPreviewPngPath
                << QStringLiteral(" pointCloud=") << result.pointCloudPlyPath;

            --remainingCaptures;
            if (remainingCaptures <= 0) {
                loop.quit();
                return;
            }

            // 连续多次采集时短暂间隔，避免 pipeline 尚未就绪
            QTimer::singleShot(200, &worker, [&worker, &options, &remainingCaptures]() {
                scan_tracking::orbbec_gemini::OrbbecCaptureRequest request;
                request.requestId = static_cast<quint64>(options.captureCount - remainingCaptures + 1);
                request.timeoutMs = options.captureTimeoutMs;
                request.saveToDisk = true;
                worker.performCapture(request);
            });
        });

    worker.startWorker(options.openConfig);
    loop.exec();
    worker.stopWorker();
    return exitCode;
}
