/**
 * @file lb_pose_check.cpp
 * @brief LB位姿检测实现文件
 *
 * 实现传统的LB位姿检测算法，包括相机标定参数配置、立体匹配、三维重建
 * 和基于模板的位姿估计等功能。
 */

#include "scan_tracking/tracking/lb_pose_check.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLoggingCategory>

#include <opencv2/opencv.hpp>

#include "TR_Mark_3D_Recon.h"
#include "TR_Mark_Track.h"
#include "AppConfig.h"

/// LB位姿检测日志类别
Q_LOGGING_CATEGORY(LOG_LB_POSE, "tracking.lb_pose")

namespace scan_tracking::tracking {
namespace {

namespace fs = std::filesystem;

/// 默认数据根目录
// @return 默认的测试数据根路径
QString defaultDataRoot()
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("data/LB"));
}

/// 将配置值规范化为默认值
// @param value 用户配置的值
// @param fallback 默认值
// @return 规范化后的值（若输入为空则返回默认值）
QString normalizeOrDefault(const QString& value, const QString& fallback)
{
    return value.trimmed().isEmpty() ? fallback : value.trimmed();
}

/// 将QString路径转换为OpenCV兼容的std::string路径
// @param path Qt格式路径
// @return 转换为本地操作系统路径格式的std::string
std::string toCvPath(const QString& path)
{
    return QDir::toNativeSeparators(path).toStdString();
}

/// 检查RT矩阵是否有效
// @param rt 4x4变换矩阵
// @return 有效返回true，否则返回false
bool isValidRt(const cv::Mat& rt)
{
    return !rt.empty() && rt.rows == 4 && rt.cols == 4 && rt.type() == CV_64F;
}

/// 将OpenCV矩阵转换为16元素数组
// @param rt 4x4变换矩阵
// @return 标准化的16元素数组
std::array<double, 16> toRtArray(const cv::Mat& rt)
{
    std::array<double, 16> values = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    if (!isValidRt(rt)) {
        return values;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            values[static_cast<std::size_t>(row * 4 + col)] = rt.at<double>(row, col);
        }
    }
    return values;
}

/// 计算平移向量的模长（毫米）
// @param rt 4x4变换矩阵
// @return 平移向量的欧几里得距离
double translationNormMm(const cv::Mat& rt)
{
    if (!isValidRt(rt)) {
        return 0.0;
    }
    const double tx = rt.at<double>(0, 3);
    const double ty = rt.at<double>(1, 3);
    const double tz = rt.at<double>(2, 3);
    return std::sqrt(tx * tx + ty * ty + tz * tz);
}

/// 规范化路径显示文本
// @param path 原始路径
// @return 规范化后的路径文本
QString summarizePath(const QString& path)
{
    return QDir::toNativeSeparators(path.trimmed());
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

}  // namespace

/// 执行传统的LB位姿检测算法
//
// 该函数执行完整的位姿检测流程：
// 1. 加载左右相机图像
// 2. 配置相机标定参数
// 3. 执行三维重建获取点云
// 4. 加载模板文件并构建哈希索引
// 5. 执行模板匹配计算位姿
//
// @param config LB位姿检测配置参数
// @return 位姿检测结果结构体
PoseCheckResult runLegacyLbPoseCheck(const scan_tracking::common::LbPoseConfig& config)
{
    PoseCheckResult result;

    const QString trackConfigPath = resolveTrackConfigPath(config);
    if (!QFileInfo::exists(trackConfigPath)) {
        result.resultCode = 9;
        result.message = QStringLiteral("track_config.ini 不存在：%1").arg(summarizePath(trackConfigPath));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    const std::wstring nativeTrackConfigPath =
        QDir::toNativeSeparators(trackConfigPath).toStdWString();
    if (!AppConfig::Instance().Load(nativeTrackConfigPath.c_str())) {
        result.resultCode = 9;
        result.message = QStringLiteral("track_config.ini 解析失败：%1").arg(summarizePath(trackConfigPath));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }
    const AppConfig& trackCfg = AppConfig::Instance();

    const QString dataRootText = normalizeOrDefault(config.dataRoot, defaultDataRoot());
    const QString leftPatternText = normalizeOrDefault(
        config.leftPattern,
        QDir(dataRootText).filePath(QStringLiteral("L/*.bmp")));
    const QString rightPatternText = normalizeOrDefault(
        config.rightPattern,
        QDir(dataRootText).filePath(QStringLiteral("R/*.bmp")));
    QString templateText = config.templateFile.trimmed();
    if (templateText.isEmpty()) {
        templateText = QString::fromLocal8Bit(trackCfg.paths.template_points.c_str()).trimmed();
    }
    if (templateText.isEmpty()) {
        templateText = QDir(dataRootText).filePath(QStringLiteral("template_for_scanner_ori.txt"));
    }

    qInfo(LOG_LB_POSE).noquote()
        << QStringLiteral("LB 位姿配置：")
        << QStringLiteral(" trackConfig=") << summarizePath(trackConfigPath)
        << QStringLiteral(" dataRoot=") << summarizePath(dataRootText)
        << QStringLiteral(" leftPattern=") << summarizePath(leftPatternText)
        << QStringLiteral(" rightPattern=") << summarizePath(rightPatternText)
        << QStringLiteral(" templateFile=") << summarizePath(templateText);

    // 检查数据根目录是否存在
    const QFileInfo dataRootInfo(dataRootText);
    if (!dataRootInfo.exists()) {
        result.resultCode = 9;
        result.message = QStringLiteral("LB 位姿数据根目录不存在：%1").arg(summarizePath(dataRootText));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    // 使用OpenCV glob查找匹配的图像文件
    std::vector<cv::String> leftFiles;
    std::vector<cv::String> rightFiles;
    cv::glob(toCvPath(leftPatternText), leftFiles, false);
    cv::glob(toCvPath(rightPatternText), rightFiles, false);

    // 检查左右相机图像是否都存在
    if (leftFiles.empty() || rightFiles.empty()) {
        result.resultCode = 5;
        result.message = QStringLiteral("LB 位姿输入图像缺失：左=%1，右=%2")
                          .arg(static_cast<int>(leftFiles.size()))
                          .arg(static_cast<int>(rightFiles.size()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    // 检查左右图像数量是否匹配
    if (leftFiles.size() != rightFiles.size()) {
        result.resultCode = 9;
        result.message = QStringLiteral("LB 位姿图像数量不匹配：左=%1，右=%2")
                          .arg(static_cast<int>(leftFiles.size()))
                          .arg(static_cast<int>(rightFiles.size()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    // 读取第一对立体图像
    cv::Mat leftImage = cv::imread(leftFiles.front(), cv::IMREAD_UNCHANGED);
    cv::Mat rightImage = cv::imread(rightFiles.front(), cv::IMREAD_UNCHANGED);
    if (leftImage.empty() || rightImage.empty()) {
        result.resultCode = 5;
        result.message = QStringLiteral("LB 位姿加载立体图像失败：左=%1 右=%2")
                          .arg(QString::fromStdString(leftFiles.front()))
                          .arg(QString::fromStdString(rightFiles.front()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    result.inputPointCount = 0;
    result.invoked = true;

    try {
        TR_INSPECT_3D_Recon_Marker recon;
        if (recon.Set_Calib_Config(
                trackCfg.recon.I1,
                trackCfg.recon.D1,
                trackCfg.recon.E1,
                trackCfg.recon.I2,
                trackCfg.recon.D2,
                trackCfg.recon.E2) != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB 位姿设置标定失败（track_config.ini [Recon]）。");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }
        if (recon.Set_2D_Config(
                trackCfg.recon.epipolar_threshold,
                trackCfg.recon.min_z_range,
                trackCfg.recon.max_z_range,
                trackCfg.recon.max_reproj_err,
                trackCfg.recon.max_ratio) != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB 位姿设置重建参数失败（track_config.ini [Recon]）。");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        const int reconResult = recon.Get_3D_Recon_Marker(leftImage, rightImage);
        if (reconResult != 0 || recon.frame_3d_points.empty()) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB 位姿 3D 重建失败，代码=%1，点数=%2")
                              .arg(reconResult)
                              .arg(static_cast<int>(recon.frame_3d_points.size()));
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        result.inputPointCount = static_cast<int>(recon.frame_3d_points.size());

        // 创建哈希索引对象并配置查询参数
        FastGeoHash geoHash(trackCfg.geo_hash.max_distance, trackCfg.geo_hash.min_distance);
        const float angleToleranceDeg =
            config.angleToleranceDeg > 0.0f ? config.angleToleranceDeg : 2.0f;
        const float lengthTolerance =
            config.lengthTolerance > 0.0f ? config.lengthTolerance : 0.5f;
        const float minPercent = config.minPercent > 0.0f ? config.minPercent : 0.5f;
        if (geoHash.set_template_config(trackCfg.geo_hash.min_distance, trackCfg.geo_hash.max_distance) != 0 ||
            geoHash.set_query_config(angleToleranceDeg, minPercent, lengthTolerance) != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB 位姿设置哈希配置失败。");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        // 加载模板文件
        std::string templatePathBytes = toCvPath(templateText);
        if (geoHash.read_template_pnts(templatePathBytes.data()) != 0) {
            result.resultCode = 9;
            result.message = QStringLiteral("LB 位姿模板文件未找到或无效：%1")
                              .arg(summarizePath(templateText));
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        // 构建哈希索引
        if (geoHash.build() != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB 位姿哈希构建失败。");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        cv::Mat scanToMarkerRt = trackCfg.geo_hash.scan_to_marker_RT.clone();
        if (geoHash.set_scan_to_marker_RT(scanToMarkerRt) != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB 位姿设置 scan_to_marker_RT 失败（track_config.ini [GeoHash]）。");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        const int trackResult = geoHash.Get_Track_Pose(
            recon.frame_3d_points,
            angleToleranceDeg,
            minPercent);

        // 检查位姿估计结果
        if (trackResult != 0 || !isValidRt(geoHash.Rt_global)) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB 位姿 Get_Track_Pose 失败，代码=%1").arg(trackResult);
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        // 位姿检测成功，填充结果
        result.success = true;
        result.resultCode = 1;
        result.poseDeviationMm = translationNormMm(geoHash.Rt_global);
        result.rt = toRtArray(geoHash.Rt_global);
        result.message = QStringLiteral("LB 位姿检测成功。");
        qInfo(LOG_LB_POSE).noquote()
            << QStringLiteral("LB 位姿检测成功")
            << QStringLiteral(" inputPoints=") << result.inputPointCount
            << QStringLiteral(" deviationMm=") << result.poseDeviationMm;
        return result;
    } catch (const std::exception& ex) {
        // 捕获标准异常
        result.resultCode = 7;
        result.message = QStringLiteral("LB 位姿异常：%1").arg(QString::fromLocal8Bit(ex.what()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    } catch (...) {
        // 捕获未知异常
        result.resultCode = 7;
        result.message = QStringLiteral("LB 位姿未知异常。");
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }
}

}  // namespace scan_tracking::tracking
