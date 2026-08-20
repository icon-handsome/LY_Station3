#include <memory>

#include <QCoreApplication>
#include <QtNetwork/QNetworkProxy>
#include <QtNetwork/QNetworkProxyFactory>

#include "scan_tracking/app/console_runtime.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/common/win_crash_dump.h"
#include "scan_tracking/vision/hik_mvs_sdk_runtime.h"

#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    // Install as early as possible so SEH / std::terminate can write MiniDump
    // under <exeDir>/crash_dumps even when WER LocalDumps misses the crash.
    scan_tracking::common::installWinCrashDumpHandler();

    QCoreApplication app(argc, argv);
    QNetworkProxyFactory::setUseSystemConfiguration(false);
    QNetworkProxy::setApplicationProxy(QNetworkProxy::NoProxy);
    app.setApplicationName("Scan Tracking");
    app.setOrganizationName("ScanTracking");

    const bool customLoggerEnabled =
        !qEnvironmentVariableIsSet("SCAN_TRACKING_DISABLE_CUSTOM_LOGGER");
    if (customLoggerEnabled) {
        scan_tracking::common::Logger::initialize(
            QCoreApplication::applicationDirPath() + QStringLiteral("/logs"));
    }
    // 工位一依赖 start.bat 设置 GenTL；此处在进程内提前补齐，避免 IDE/直接双击启动时 CXP 枚举失败。
    scan_tracking::vision::ensureHikGenTlEnvironment();
    scan_tracking::common::ConfigManager::initialize();

    const int exit_code = [&app]() {
        auto runtime = std::make_unique<scan_tracking::app::ConsoleRuntime>(app);
        return runtime->run();
    }();

    scan_tracking::common::ConfigManager::cleanup();
    if (customLoggerEnabled) {
        scan_tracking::common::Logger::cleanup();
    }
    return exit_code;
}
