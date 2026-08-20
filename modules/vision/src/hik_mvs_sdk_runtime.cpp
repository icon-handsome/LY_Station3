#include "scan_tracking/vision/hik_mvs_sdk_runtime.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QMutex>
#include <QtCore/QMutexLocker>

#include <qdebug.h>

#include "MvCameraControl.h"

namespace scan_tracking {
namespace vision {

namespace {

QMutex g_sdkMutex;
int g_sdkRefCount = 0;

bool directoryHasCxpCti(const QString& dir)
{
    if (dir.isEmpty()) {
        return false;
    }
    return QFileInfo::exists(QDir(dir).filePath(QStringLiteral("MvFGProducerCXP.cti")));
}

}  // namespace

void ensureHikGenTlEnvironment()
{
    // 与工位一 start.bat 对齐：CXP 依赖 GenTL Producer（MvFGProducerCXP.cti）。
    // 必须在 MV_CC_Initialize 之前设置，否则 EnumDevices(MV_GENTL_CXP_DEVICE) 会返回 0x800000FF。
    static bool done = false;
    if (done) {
        return;
    }
    done = true;

    const QString mvsRuntime = QDir::toNativeSeparators(
        QStringLiteral("C:/Program Files (x86)/Common Files/MVS/Runtime/Win64_x64"));

    QStringList preferredDirs;
    preferredDirs << mvsRuntime;

    if (QCoreApplication::instance() != nullptr) {
        const QString appDir = QDir::toNativeSeparators(QCoreApplication::applicationDirPath());
        if (!appDir.isEmpty()) {
            preferredDirs << appDir;
            preferredDirs << QDir::toNativeSeparators(
                QDir(appDir).filePath(QStringLiteral("hik_mvs_runtime")));
        }
    }

    QStringList gentlDirs;
    for (const QString& dir : preferredDirs) {
        if (directoryHasCxpCti(dir) && !gentlDirs.contains(dir, Qt::CaseInsensitive)) {
            gentlDirs << dir;
        }
    }

    const QByteArray existingGentl = qgetenv("GENICAM_GENTL64_PATH");
    QString merged = QString::fromLocal8Bit(existingGentl).trimmed();
    for (const QString& dir : gentlDirs) {
        if (merged.isEmpty()) {
            merged = dir;
        } else if (!merged.contains(dir, Qt::CaseInsensitive)) {
            merged = dir + QLatin1Char(';') + merged;
        }
    }
    if (merged.isEmpty()) {
        merged = mvsRuntime;
    }

    qputenv("GENICAM_GENTL64_PATH", merged.toLocal8Bit());

    QString pathEnv = QString::fromLocal8Bit(qgetenv("PATH"));
    for (const QString& dir : preferredDirs) {
        if (dir.isEmpty()) {
            continue;
        }
        if (!pathEnv.contains(dir, Qt::CaseInsensitive)) {
            pathEnv = dir + QLatin1Char(';') + pathEnv;
        }
    }
    qputenv("PATH", pathEnv.toLocal8Bit());

    bool ctiFound = false;
    for (const QString& dir : merged.split(QLatin1Char(';'), Qt::SkipEmptyParts)) {
        if (directoryHasCxpCti(dir.trimmed())) {
            ctiFound = true;
            break;
        }
    }

    qInfo().noquote() << QStringLiteral("[MVS] GENICAM_GENTL64_PATH=%1 MvFGProducerCXP.cti=%2")
                             .arg(merged, ctiFound ? QStringLiteral("found") : QStringLiteral("MISSING"));
    if (!ctiFound) {
        qWarning().noquote() << QStringLiteral(
            "[MVS] 未找到 MvFGProducerCXP.cti。请安装海康 MVS，或将 CTI 放到 exe 目录 / "
            "hik_mvs_runtime，并用 start.bat 启动（与工位一相同）。");
    }
}

bool acquireHikMvsSdk(QString* errorMessage)
{
    QMutexLocker locker(&g_sdkMutex);
    if (g_sdkRefCount > 0) {
        ++g_sdkRefCount;
        return true;
    }

    ensureHikGenTlEnvironment();

    const int result = MV_CC_Initialize();
    if (result != MV_OK) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("MVS SDK 初始化失败，错误码=0x%1")
                                .arg(static_cast<quint32>(result), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    ++g_sdkRefCount;
    return true;
}

void releaseHikMvsSdk()
{
    QMutexLocker locker(&g_sdkMutex);
    if (g_sdkRefCount <= 0) {
        return;
    }
    --g_sdkRefCount;
    if (g_sdkRefCount == 0) {
        MV_CC_Finalize();
    }
}

}  // namespace vision
}  // namespace scan_tracking
