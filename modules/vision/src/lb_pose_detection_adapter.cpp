#include "scan_tracking/vision/lb_pose_detection_adapter.h"

#include <QtCore/QByteArray>
#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <opencv2/opencv.hpp>

#include "AppConfig.h"
#include "TR_Mark_3D_Recon.h"
#include "TR_Mark_Track.h"

#include "scan_tracking/common/config_manager.h"

namespace scan_tracking {
namespace vision {

// vision.lb_pose 打印已屏蔽
// Q_LOGGING_CATEGORY(LOG_LB_POSE, "vision.lb_pose")

namespace {

LbPoseResult makeFailure(const QString& message)
{
    LbPoseResult result;
    result.invoked = true;
    result.success = false;
    result.message = message;
    return result;
}

QString resolveTrackConfigPath(const scan_tracking::common::LbPoseConfig& config)
{
    const QString configured = config.trackConfigFile.trimmed();
    if (!configured.isEmpty()) {
        return QDir::toNativeSeparators(QFileInfo(configured).absoluteFilePath());
    }

    const QStringList candidates = {
        QDir(QCoreApplication::applicationDirPath())
            .absoluteFilePath(QStringLiteral("third_party/LB/track_config.ini")),
        QDir(QCoreApplication::applicationDirPath())
            .absoluteFilePath(QStringLiteral("../third_party/LB/track_config.ini")),
    };

    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) {
            return QDir::toNativeSeparators(QFileInfo(candidate).absoluteFilePath());
        }
    }

    return candidates.front();
}

bool loadTrackConfig(const scan_tracking::common::LbPoseConfig& config, QString* errorMessage)
{
    const QString trackConfigPath = resolveTrackConfigPath(config);
    if (!QFileInfo::exists(trackConfigPath)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("track_config.ini 不存在：%1").arg(trackConfigPath);
        }
        return false;
    }

    const std::wstring nativePath = QDir::toNativeSeparators(trackConfigPath).toStdWString();
    if (!AppConfig::Instance().Load(nativePath.c_str())) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("track_config.ini 解析失败：%1").arg(trackConfigPath);
        }
        return false;
    }

    return true;
}

cv::Mat frameToGrayMat(const HikMonoFrame& frame)
{
    if (!frame.isValid() || frame.pixels == nullptr || frame.width <= 0 || frame.height <= 0) {
        return {};
    }
    if (frame.stride < frame.width) {
        return {};
    }

    cv::Mat view(frame.height, frame.width, CV_8UC1, frame.pixels->data(), static_cast<std::size_t>(frame.stride));
    return view.clone();
}

PoseMatrix4x4 toPoseMatrix(const cv::Mat& rt, const QString& sourceCameraKey, quint64 frameId)
{
    PoseMatrix4x4 pose;
    if (rt.empty() || rt.rows != 4 || rt.cols != 4 || rt.type() != CV_64F) {
        return pose;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            pose.values[static_cast<std::size_t>(row * 4 + col)] = static_cast<float>(rt.at<double>(row, col));
        }
    }
    pose.frameId = frameId;
    pose.timestampMs = QDateTime::currentMSecsSinceEpoch();
    pose.sourceCameraKey = sourceCameraKey;
    pose.valid = true;
    return pose;
}

QString resolveLbTemplatePath(
    const scan_tracking::common::LbPoseConfig& config,
    const AppConfig& trackConfig)
{
    QStringList candidates;
    const auto addCandidate = [&candidates](const QString& path) {
        const QString trimmed = path.trimmed();
        if (trimmed.isEmpty()) {
            return;
        }
        const QString native = QDir::toNativeSeparators(QFileInfo(trimmed).absoluteFilePath());
        if (!candidates.contains(native)) {
            candidates.push_back(native);
        }
    };

    addCandidate(config.templateFile);
    addCandidate(QString::fromLocal8Bit(trackConfig.paths.template_points.c_str()));

    const QString exeDir = QCoreApplication::applicationDirPath();
    addCandidate(QDir(exeDir).filePath(QStringLiteral("third_party/LB/template_for_scanner_ori.txt")));
    addCandidate(QDir(exeDir).filePath(QStringLiteral("data/LB/template_for_scanner_ori.txt")));

    const QString dataRoot = config.dataRoot.trimmed();
    if (!dataRoot.isEmpty()) {
        addCandidate(QDir(dataRoot).filePath(QStringLiteral("template_for_scanner_ori.txt")));
    }

    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) {
            return candidate;
        }
    }
    return candidates.isEmpty() ? QString() : candidates.front();
}

QString buildTemplatePath(
    const scan_tracking::common::LbPoseConfig& config,
    const AppConfig& trackConfig)
{
    return resolveLbTemplatePath(config, trackConfig);
}

QString formatPoint3fCsv(const cv::Point3f& point)
{
    return QStringLiteral("%1,%2,%3")
        .arg(static_cast<double>(point.x), 0, 'f', 6)
        .arg(static_cast<double>(point.y), 0, 'f', 6)
        .arg(static_cast<double>(point.z), 0, 'f', 6);
}

QString formatPoint2fCsvZeroZ(const cv::Point2f& point)
{
    return QStringLiteral("%1,%2,0.0")
        .arg(static_cast<double>(point.x), 0, 'f', 6)
        .arg(static_cast<double>(point.y), 0, 'f', 6);
}

QString formatPoint3fSpace(const cv::Point3f& point)
{
    return QStringLiteral("%1 %2 %3")
        .arg(static_cast<double>(point.x), 0, 'f', 8)
        .arg(static_cast<double>(point.y), 0, 'f', 8)
        .arg(static_cast<double>(point.z), 0, 'f', 8);
}

QString formatCvMat4x8Block(const QString& title, const cv::Mat& matrix)
{
    QString text = title + QStringLiteral("\n");
    if (matrix.empty() || matrix.rows != 4 || matrix.cols != 4 || matrix.type() != CV_64F) {
        text += QStringLiteral("(invalid matrix)\n");
        return text;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            text += QStringLiteral("%1 ").arg(matrix.at<double>(row, col), 8, 'f', 8);
        }
        text += QStringLiteral("\n");
    }
    return text;
}

QString buildLbDiagnosticText(
    const HikMonoFrame& leftFrame,
    const HikMonoFrame& rightFrame,
    const TR_INSPECT_3D_Recon_Marker& recon,
    const FastGeoHash& geoHash)
{
    QString text;
    text += QStringLiteral("# LB pose detection diagnostic\n");
    text += QStringLiteral("cxpLeft.frameId=") + QString::number(leftFrame.frameId) + QStringLiteral("\n");
    text += QStringLiteral("cxpLeft.timestampMs=") + QString::number(leftFrame.timestampMs) + QStringLiteral("\n");
    text += QStringLiteral("cxpLeft.sourceCameraKey=") + leftFrame.sourceCameraKey + QStringLiteral("\n");
    text += QStringLiteral("cxpRight.frameId=") + QString::number(rightFrame.frameId) + QStringLiteral("\n");
    text += QStringLiteral("cxpRight.timestampMs=") + QString::number(rightFrame.timestampMs) + QStringLiteral("\n");
    text += QStringLiteral("cxpRight.sourceCameraKey=") + rightFrame.sourceCameraKey + QStringLiteral("\n");

    text += QStringLiteral("\n三维重建标记点数量: ")
        + QString::number(recon.frame_3d_points.size()) + QStringLiteral("\n");
    for (const cv::Point3f& point : recon.frame_3d_points) {
        text += formatPoint3fCsv(point) + QStringLiteral("\n");
    }

    text += QStringLiteral("\n对应点数量: ")
        + QString::number(geoHash.filtered_frame_3d_points.size()) + QStringLiteral("\n");
    text += QStringLiteral("标记点:\n");
    for (const cv::Point3f& point : geoHash.filtered_frame_3d_points) {
        text += formatPoint3fSpace(point) + QStringLiteral("\n");
    }
    text += QStringLiteral("\n模板点:\n");
    for (const cv::Point3f& point : geoHash.template_pnts) {
        text += formatPoint3fSpace(point) + QStringLiteral("\n");
    }
    text += QStringLiteral("\n");
    text += formatCvMat4x8Block(QStringLiteral("scan_to_marker_RT is:"), geoHash.scan_to_marker_RT);
    text += formatCvMat4x8Block(
        QStringLiteral("Realtime Rt_global (from Scanner to Vision) is:"), geoHash.Rt_global);
    return text;
}

void logLbDiagnosticText(const QString& diagnosticText)
{
    // 诊断文本写入 LbPoseResult.diagnosticText，此处不依赖已删除的导出开关。
    Q_UNUSED(diagnosticText);
}

}  // namespace

LbPoseResult runLbPoseDetection(
    const HikMonoFrame& leftFrame,
    const HikMonoFrame& rightFrame,
    const scan_tracking::common::LbPoseConfig& config)
{
    LbPoseResult result;
    result.invoked = true;
    result.leftImageWidth = leftFrame.width;
    result.leftImageHeight = leftFrame.height;
    result.rightImageWidth = rightFrame.width;
    result.rightImageHeight = rightFrame.height;

    if (!leftFrame.isValid() || !rightFrame.isValid()) {
        return makeFailure(QStringLiteral("LB 位姿检测需要两个有效的灰度图像帧。"));
    }

    QString trackConfigError;
    if (!loadTrackConfig(config, &trackConfigError)) {
        return makeFailure(trackConfigError);
    }
    const AppConfig& trackCfg = AppConfig::Instance();

    const cv::Mat leftImage = frameToGrayMat(leftFrame);
    const cv::Mat rightImage = frameToGrayMat(rightFrame);
    if (leftImage.empty() || rightImage.empty()) {
        return makeFailure(QStringLiteral("将海康图像帧转换为 cv::Mat 灰度图像失败。"));
    }

    try {
        TR_INSPECT_3D_Recon_Marker recon;
        if (recon.Set_Calib_Config(
                trackCfg.recon.I1,
                trackCfg.recon.D1,
                trackCfg.recon.E1,
                trackCfg.recon.I2,
                trackCfg.recon.D2,
                trackCfg.recon.E2) != 0) {
            return makeFailure(QStringLiteral("配置 LB 立体标定失败（track_config.ini [Recon]）。"));
        }
        if (recon.Set_2D_Config(
                trackCfg.recon.epipolar_threshold,
                trackCfg.recon.min_z_range,
                trackCfg.recon.max_z_range,
                trackCfg.recon.max_reproj_err,
                trackCfg.recon.max_ratio) != 0) {
            return makeFailure(QStringLiteral("配置 LB 二维重建参数失败（track_config.ini [Recon]）。"));
        }

        if (recon.Get_3D_Recon_Marker(const_cast<cv::Mat&>(leftImage), const_cast<cv::Mat&>(rightImage)) != 0) {
            return makeFailure(QStringLiteral("TR_INSPECT_3D_Recon_Marker 重建立体点云失败。"));
        }

        FastGeoHash geoHash(trackCfg.geo_hash.max_distance, trackCfg.geo_hash.min_distance);
        if (geoHash.set_template_config(trackCfg.geo_hash.min_distance, trackCfg.geo_hash.max_distance) != 0) {
            return makeFailure(QStringLiteral("设置 LB 模板配置失败（track_config.ini [GeoHash]）。"));
        }
        const float angleToleranceDeg =
            config.angleToleranceDeg > 0.0f ? config.angleToleranceDeg : trackCfg.geo_hash.angle_tolerance_deg;
        const float lengthTolerance =
            config.lengthTolerance > 0.0f ? config.lengthTolerance : trackCfg.geo_hash.length_tolerance;
        const float minPercent = config.minPercent > 0.0f ? config.minPercent : trackCfg.geo_hash.min_percent;
        if (geoHash.set_query_config(
                angleToleranceDeg,
                minPercent,
                lengthTolerance) != 0) {
            return makeFailure(QStringLiteral("设置 LB 查询配置失败（track_config.ini [GeoHash]）。"));
        }

        const QString templatePath = buildTemplatePath(config, trackCfg);
        if (templatePath.trimmed().isEmpty() || !QFileInfo::exists(templatePath)) {
            return makeFailure(
                QStringLiteral("LB 模板不存在：%1（请配置 [LbPose] templateFile=third_party/LB/template_for_scanner_ori.txt）")
                    .arg(templatePath.isEmpty() ? QStringLiteral("<未配置>") : templatePath));
        }

        const QByteArray templateBytes =
            QFile::encodeName(QDir::toNativeSeparators(templatePath));
        if (geoHash.read_template_pnts(templateBytes.constData()) != 0) {
            return makeFailure(QStringLiteral("从 %1 加载 LB 模板点云失败。").arg(templatePath));
        }
        if (geoHash.build() != 0) {
            return makeFailure(QStringLiteral("构建 LB 几何哈希表失败。"));
        }

        cv::Mat scanToMarkerRt = trackCfg.geo_hash.scan_to_marker_RT.clone();
        if (geoHash.set_scan_to_marker_RT(scanToMarkerRt) != 0) {
            return makeFailure(QStringLiteral("配置 LB scan_to_marker_RT 失败（track_config.ini [GeoHash]）。"));
        }

        const int reconPointCount = static_cast<int>(recon.frame_3d_points.size());
        const int trackResult = geoHash.Get_Track_Pose(
            recon.frame_3d_points,
            angleToleranceDeg,
            minPercent);
        if (trackResult != 0) {
            // vision.lb_pose 打印已屏蔽
            // if (scan_tracking::common::segmentCaptureExportEnabled()) {
            //     qWarning(LOG_LB_POSE).noquote()
            //         << QStringLiteral("LB Get_Track_Pose 失败 code=%1").arg(trackResult)
            //         << QStringLiteral("三维重建点数=") << geoHash.track_pose_last_recon_count
            //         << QStringLiteral("邻域滤波后=") << geoHash.track_pose_last_neighbor_filtered_count
            //         << QStringLiteral("模板匹配点数=") << geoHash.track_pose_last_matched_count
            //         << QStringLiteral("最少需要=") << geoHash.track_pose_last_min_pose_points
            //         << QStringLiteral("当前重建点数(调用前)=") << reconPointCount;
            // }
            Q_UNUSED(reconPointCount);
            return makeFailure(QStringLiteral("LB 跟踪返回错误代码 %1。").arg(trackResult));
        }

        result.diagnosticText = buildLbDiagnosticText(leftFrame, rightFrame, recon, geoHash);
        logLbDiagnosticText(result.diagnosticText);

        result.success = true;
        result.message = QStringLiteral("LB 位姿检测成功完成。");
        result.framePointCount = static_cast<int>(recon.frame_3d_points.size());
        result.poseMatrix = toPoseMatrix(geoHash.Rt_global, QStringLiteral("lb_pose_detection"), 0);
        if (!result.poseMatrix.isValid()) {
            return makeFailure(QStringLiteral("LB 位姿检测产生了无效的 Rt_global 矩阵。"));
        }
        return result;
    } catch (const std::exception& ex) {
        return makeFailure(QStringLiteral("LB 位姿检测抛出异常：%1").arg(QString::fromLocal8Bit(ex.what())));
    } catch (...) {
        return makeFailure(QStringLiteral("LB 位姿检测抛出未知异常。"));
    }
}

}  // namespace vision
}  // namespace scan_tracking
