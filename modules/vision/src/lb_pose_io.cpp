#include "scan_tracking/vision/lb_pose_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QLoggingCategory>
#include <QTextStream>

Q_LOGGING_CATEGORY(LOG_LB_POSE_IO, "vision.lb_pose_io")

namespace scan_tracking::vision {

namespace {

bool writeTextFile(const QString& absolutePath, const QString& content, QString* errorMessage)
{
    QFileInfo fileInfo(absolutePath);
    if (!QDir().mkpath(fileInfo.absolutePath())) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法创建目录：%1").arg(fileInfo.absolutePath());
        }
        return false;
    }

    QFile file(absolutePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法写入：%1").arg(absolutePath);
        }
        return false;
    }

    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream << content;
    if (stream.status() != QTextStream::Ok) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("写入失败：%1").arg(absolutePath);
        }
        return false;
    }
    return true;
}

QString formatRtGlobalText(const LbPoseResult& lb)
{
    QString text;
    QTextStream stream(&text);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(8);

    stream << QStringLiteral("# LB Rt_global (row-major 4x4), IPC uses as T0 for Mech cloud\n");
    stream << QStringLiteral("# invoked=") << (lb.invoked ? 1 : 0)
           << QStringLiteral(" success=") << (lb.success ? 1 : 0)
           << QStringLiteral(" valid=") << (lb.poseMatrix.isValid() ? 1 : 0)
           << QStringLiteral(" framePointCount=") << lb.framePointCount << QLatin1Char('\n');
    stream << QStringLiteral("# source=") << lb.poseMatrix.sourceCameraKey
           << QStringLiteral(" frameId=") << lb.poseMatrix.frameId
           << QStringLiteral(" timestampMs=") << lb.poseMatrix.timestampMs << QLatin1Char('\n');
    if (!lb.message.trimmed().isEmpty()) {
        stream << QStringLiteral("# message=") << lb.message.trimmed() << QLatin1Char('\n');
    }

    if (lb.poseMatrix.isValid()) {
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                if (col > 0) {
                    stream << QLatin1Char(' ');
                }
                stream << lb.poseMatrix.values[static_cast<std::size_t>(row * 4 + col)];
            }
            stream << QLatin1Char('\n');
        }
    } else {
        stream << QStringLiteral("# matrix unavailable\n");
    }

    return text;
}

}  // namespace

QString buildSegmentLbPoseDir(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString pointDir = scan_tracking::common::capturePointDirectory(
        configuredRoot, pathId, deviceTag, segmentIndex);
    if (pointDir.isEmpty()) {
        qWarning(LOG_LB_POSE_IO).noquote()
            << QStringLiteral("无法创建点位目录(lb) pathId=") << pathId
            << QStringLiteral(" device=") << deviceTag
            << QStringLiteral(" point=") << segmentIndex;
        return QString();
    }
    return pointDir;
}

QString buildSegmentLbPoseMatrixPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString dir = buildSegmentLbPoseDir(configuredRoot, pathId, deviceTag, segmentIndex);
    if (dir.isEmpty()) {
        return QString();
    }
    return QDir(dir).absoluteFilePath(
        scan_tracking::common::buildCaptureArtifactFileName(
            pathId, deviceTag, QStringLiteral("rt_global"), segmentIndex, QStringLiteral("txt")));
}

QString buildSegmentLbPoseDiagnosticPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString dir = buildSegmentLbPoseDir(configuredRoot, pathId, deviceTag, segmentIndex);
    if (dir.isEmpty()) {
        return QString();
    }
    return QDir(dir).absoluteFilePath(
        scan_tracking::common::buildCaptureArtifactFileName(
            pathId, deviceTag, QStringLiteral("diagnostic"), segmentIndex, QStringLiteral("txt")));
}

bool saveLbPoseResultToDisk(
    const LbPoseResult& lb,
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex,
    QString* errorMessage)
{
    if (!lb.invoked) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("LB 未调用，跳过落盘");
        }
        return false;
    }

    const QString matrixPath =
        buildSegmentLbPoseMatrixPath(configuredRoot, pathId, deviceTag, segmentIndex);
    if (matrixPath.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("LB 落盘路径无效");
        }
        return false;
    }

    if (!writeTextFile(matrixPath, formatRtGlobalText(lb), errorMessage)) {
        qWarning(LOG_LB_POSE_IO).noquote()
            << QStringLiteral("保存 rt_global 失败：")
            << (errorMessage != nullptr ? *errorMessage : matrixPath);
        return false;
    }

    if (!lb.diagnosticText.trimmed().isEmpty()) {
        const QString diagPath =
            buildSegmentLbPoseDiagnosticPath(configuredRoot, pathId, deviceTag, segmentIndex);
        QString diagError;
        if (diagPath.isEmpty() ||
            !writeTextFile(diagPath, lb.diagnosticText, &diagError)) {
            qWarning(LOG_LB_POSE_IO).noquote()
                << QStringLiteral("保存 diagnostic 失败：") << diagError
                << QStringLiteral(" path=") << diagPath;
            // 矩阵已落盘；诊断失败不阻断主流程
        }
    }

    qInfo(LOG_LB_POSE_IO).noquote()
        << QStringLiteral("LB 位姿已落盘：") << matrixPath
        << QStringLiteral(" success=") << lb.success
        << QStringLiteral(" valid=") << lb.poseMatrix.isValid();
    return true;
}

}  // namespace scan_tracking::vision
