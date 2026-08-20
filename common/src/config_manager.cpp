/**
 * @file config_manager.cpp
 * @brief ConfigManager 实现：加载 config.ini、工位 profile 与 scan_paths JSON
 *
 * 配置文件查找顺序（projectRootConfigPath）：
 * 1. 可执行文件同目录下的 config.ini
 * 2. 自 exe 向上三级目录（开发态 CMake 构建树中的项目根）
 *
 * 相对路径解析（resolveConfigRelativePath）：config.ini 所在目录 → exe 目录。
 */

#include "scan_tracking/common/config_manager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLoggingCategory>
#include <QStringList>

#include "scan_tracking/common/logger.h"

namespace scan_tracking {
namespace common {

Q_LOGGING_CATEGORY(LOG_CONFIG, "config")

ConfigManager* ConfigManager::s_instance = nullptr;

namespace {

/**
 * @brief 定位主配置文件 config.ini 的路径
 *
 * 优先使用与可执行文件同目录的 config.ini（部署场景）；
 * 若不存在则尝试 CMake 开发构建树中的项目根目录。
 */
QString projectRootConfigPath()
{
    const QString exeDirConfig =
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config.ini"));
    if (QFileInfo::exists(exeDirConfig)) {
        return exeDirConfig;
    }

    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        return rootDir.filePath(QStringLiteral("config.ini"));
    }
    return exeDirConfig;
}

/**
 * @brief 将配置中的相对路径解析为绝对路径
 *
 * 解析顺序：已是绝对路径则直接规范化；否则依次尝试
 * config.ini 所在目录、可执行文件目录。均不存在时仍返回 config 目录下的拼接结果。
 *
 * @param rawPath         配置项中的原始路径字符串
 * @param configFilePath  主 config.ini 的绝对路径（用作相对路径基准）
 */
QString resolveConfigRelativePath(const QString& rawPath, const QString& configFilePath)
{
    const QString trimmed = rawPath.trimmed();
    if (trimmed.isEmpty()) {
        return QString();
    }

    const QFileInfo pathInfo(trimmed);
    if (pathInfo.isAbsolute()) {
        return QDir::cleanPath(trimmed);
    }

    const QString configDirPath = QFileInfo(configFilePath).absoluteDir().filePath(trimmed);
    if (QFileInfo::exists(configDirPath)) {
        return QDir::cleanPath(configDirPath);
    }

    const QString exeDirPath = QDir(QCoreApplication::applicationDirPath()).filePath(trimmed);
    if (QFileInfo::exists(exeDirPath)) {
        return QDir::cleanPath(exeDirPath);
    }

    return QDir::cleanPath(configDirPath);
}

/**
 * @brief 确定 scan_paths JSON 文件的完整路径
 *
 * 优先级：
 * 1. StationProfile::scanPathsConfigPath（来自 [Station] 或 profileIni 覆盖）
 * 2. 默认 fallback：config/scan_paths/station3_placeholder.json
 *
 * @param stationProfile  已合并的工位 profile
 * @param configFilePath  主 config.ini 路径，用于相对路径解析
 */
QString scanPathsConfigPath(const StationProfile& stationProfile, const QString& configFilePath)
{
    if (!stationProfile.scanPathsConfigPath.trimmed().isEmpty()) {
        return resolveConfigRelativePath(stationProfile.scanPathsConfigPath, configFilePath);
    }

    const QString fallbackPath =
        resolveConfigRelativePath(QStringLiteral("config/scan_paths/station3_placeholder.json"), configFilePath);
    if (QFileInfo::exists(fallbackPath)) {
        return fallbackPath;
    }

    return fallbackPath;
}

/**
 * @brief 从 QSettings 的 [Station] 节读取字段并写入 profile
 *
 * 仅当键存在时才覆盖 profile 中对应字段，便于多层配置合并。
 * 若 profileIni 非空且 settings 含 profileIni 键，则将其路径写入 *profileIni。
 *
 * @param settings    已定位到 [Station] 组内或即将 beginGroup 的 QSettings
 * @param profile     读入目标，就地修改
 * @param profileIni  可选输出；非 nullptr 时读取 profileIni 键
 */
void applyStationSettings(QSettings& settings, StationProfile& profile, QString* profileIni)
{
    if (settings.contains(QStringLiteral("stationId"))) {
        profile.stationId = stationIdFromInt(settings.value(QStringLiteral("stationId"), 2).toInt());
    }
    if (settings.contains(QStringLiteral("stationName"))) {
        profile.stationName = settings.value(QStringLiteral("stationName"), profile.stationName).toString();
    }
    if (settings.contains(QStringLiteral("scanPathsConfigPath"))) {
        profile.scanPathsConfigPath =
            settings.value(QStringLiteral("scanPathsConfigPath"), profile.scanPathsConfigPath).toString().trimmed();
    }
    if (settings.contains(QStringLiteral("defaultWorkMode"))) {
        bool ok = false;
        const QString raw = settings.value(QStringLiteral("defaultWorkMode")).toString();
        profile.defaultWorkMode = workModeIdFromString(raw, &ok);
        if (!ok) {
            qWarning(LOG_CONFIG).noquote()
                << QStringLiteral("[Station] 未知 defaultWorkMode=") << raw
                << QStringLiteral("，已回退 Unknown");
        }
    }
    if (settings.contains(QStringLiteral("enableLoadGrasp"))) {
        profile.enableLoadGrasp = settings.value(QStringLiteral("enableLoadGrasp"), profile.enableLoadGrasp).toBool();
    }
    if (settings.contains(QStringLiteral("enableUnloadCalc"))) {
        profile.enableUnloadCalc = settings.value(QStringLiteral("enableUnloadCalc"), profile.enableUnloadCalc).toBool();
    }
    if (settings.contains(QStringLiteral("enablePoseCheck"))) {
        profile.enablePoseCheck = settings.value(QStringLiteral("enablePoseCheck"), profile.enablePoseCheck).toBool();
    }
    if (settings.contains(QStringLiteral("enableTelescopicScan"))) {
        profile.enableTelescopicScan =
            settings.value(QStringLiteral("enableTelescopicScan"), profile.enableTelescopicScan).toBool();
    }
    if (settings.contains(QStringLiteral("enableHoistAssist"))) {
        profile.enableHoistAssist = settings.value(QStringLiteral("enableHoistAssist"), profile.enableHoistAssist).toBool();
    }
    if (settings.contains(QStringLiteral("enableCollisionMonitor"))) {
        profile.enableCollisionMonitor =
            settings.value(QStringLiteral("enableCollisionMonitor"), profile.enableCollisionMonitor).toBool();
    }
    if (profileIni != nullptr && settings.contains(QStringLiteral("profileIni"))) {
        *profileIni = settings.value(QStringLiteral("profileIni")).toString().trimmed();
    }
}

}  // namespace

void ConfigManager::initialize()
{
    // 幂等：多次调用仅首次分配单例；构造函数内完成 load + loadScanPathsConfig
    if (!s_instance) {
        s_instance = new ConfigManager();
        qInfo(LOG_CONFIG) << "ConfigManager 已初始化。";
    }
}

void ConfigManager::cleanup()
{
    if (s_instance) {
        delete s_instance;
        s_instance = nullptr;
        qInfo(LOG_CONFIG) << "ConfigManager 已清理。";
    }
}

ConfigManager* ConfigManager::instance()
{
    if (!s_instance) {
        qWarning(LOG_CONFIG) << "ConfigManager::instance() 在 initialize() 之前被调用！";
    }
    return s_instance;
}

ConfigManager::ConfigManager()
{
    const QString configPath = projectRootConfigPath();
    load(configPath);

    // load() 已解析 StationProfile，据此定位 scan_paths JSON 并加载
    const QString scanPathsPath = scanPathsConfigPath(m_stationProfile, configPath);
    loadScanPathsConfig(scanPathsPath);
}

ConfigManager::~ConfigManager() = default;

const AppConfig& ConfigManager::appConfig() const { return m_appConfig; }
const LoggerConfig& ConfigManager::loggerConfig() const { return m_loggerConfig; }
const ModbusConfig& ConfigManager::modbusConfig() const { return m_modbusConfig; }
const CameraConfig& ConfigManager::cameraConfig() const { return m_cameraConfig; }
const VisionConfig& ConfigManager::visionConfig() const { return m_visionConfig; }
const FlowControlConfig& ConfigManager::flowControlConfig() const { return m_flowControlConfig; }
const TrackingConfig& ConfigManager::trackingConfig() const { return m_trackingConfig; }
const LbPoseConfig& ConfigManager::lbPoseConfig() const { return m_lbPoseConfig; }
const OrbbecGeminiConfig& ConfigManager::orbbecGeminiConfig() const { return m_orbbecGeminiConfig; }
const LivoxMid360Config& ConfigManager::livoxMid360Config() const { return m_livoxMid360Config; }
const TfminiPlusConfig& ConfigManager::tfminiPlusConfig() const { return m_tfminiPlusConfig; }
QString ConfigManager::configFilePath() const { return m_configFilePath; }
const HmiConfig& ConfigManager::hmiConfig() const { return m_hmiConfig; }
const ScanPathsConfig& ConfigManager::scanPathsConfig() const { return m_scanPathsConfig; }
const StationProfile& ConfigManager::stationProfile() const { return m_stationProfile; }

const ScanPathConfig* ConfigManager::findScanPathById(int pathId) const
{
    if (pathId <= 0) {
        return nullptr;
    }
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (path.pathId == pathId) {
            return &path;
        }
    }
    return nullptr;
}

const ScanPathConfig* ConfigManager::activeScanPath() const
{
    if (m_scanPathsConfig.activePathId > 0) {
        if (const ScanPathConfig* byId = findScanPathById(m_scanPathsConfig.activePathId)) {
            return byId;
        }
    }
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (path.enabled) {
            return &path;
        }
    }
    return nullptr;
}

int ConfigManager::activePathId() const
{
    if (const ScanPathConfig* path = activeScanPath()) {
        return path->pathId;
    }
    return 0;
}

QVector<int> ConfigManager::enabledPathIds() const
{
    QVector<int> ids;
    ids.reserve(static_cast<int>(m_scanPathsConfig.scanPaths.size()));
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (path.enabled && path.pathId > 0) {
            ids.push_back(path.pathId);
        }
    }
    return ids;
}

bool ConfigManager::setActivePathId(int pathId)
{
    if (pathId <= 0) {
        qWarning(LOG_CONFIG).noquote()
            << QStringLiteral("setActivePathId：非法 pathId=") << pathId;
        return false;
    }
    if (findScanPathById(pathId) == nullptr) {
        qWarning(LOG_CONFIG).noquote()
            << QStringLiteral("setActivePathId：pathId=") << pathId
            << QStringLiteral(" 不在 scanPaths 中");
        return false;
    }

    const int previous = m_scanPathsConfig.activePathId;
    m_scanPathsConfig.activePathId = pathId;
    if (previous != pathId) {
        const ScanPathConfig* path = findScanPathById(pathId);
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("活跃扫描路径已切换：") << previous << QStringLiteral(" -> ") << pathId
            << QStringLiteral(" name=") << (path != nullptr ? path->name : QString())
            << QStringLiteral(" algorithm=")
            << (path != nullptr ? resolvePathAlgorithm(*path) : QString())
            << QStringLiteral(" arm=") << (path != nullptr ? path->armPointCount : 0)
            << QStringLiteral(" telescopic=")
            << (path != nullptr ? path->telescopicPointCount : 0);
    }
    return true;
}

int ConfigManager::advanceToNextEnabledPath()
{
    const QVector<int> ids = enabledPathIds();
    if (ids.isEmpty()) {
        qWarning(LOG_CONFIG) << QStringLiteral("advanceToNextEnabledPath：无 enabled 路径可切换");
        return 0;
    }

    const int current = activePathId();
    int currentIndex = ids.indexOf(current);
    if (currentIndex < 0) {
        // 当前不在 enabled 列表（或聚合模式），落到第一条
        if (!setActivePathId(ids.front())) {
            return 0;
        }
        return ids.front();
    }

    const int nextId = ids.at((currentIndex + 1) % ids.size());
    if (!setActivePathId(nextId)) {
        return 0;
    }
    return nextId;
}

QString ConfigManager::activePathName() const
{
    if (const ScanPathConfig* path = activeScanPath()) {
        return path->name;
    }
    return {};
}

QString ConfigManager::resolvePathAlgorithm(const ScanPathConfig& path)
{
    const QString explicitAlgo = path.algorithm.trimmed().toLower();
    if (!explicitAlgo.isEmpty()) {
        return explicitAlgo;
    }

    const QString name = path.name.trimmed().toLower();
    if (name == QLatin1String("straight_weld") || name == QLatin1String("ring_weld") ||
        name.contains(QLatin1String("weld"))) {
        return QStringLiteral("weld_section");
    }
    if (name == QLatin1String("code_read") || name.contains(QLatin1String("code"))) {
        return QStringLiteral("code_read");
    }
    if (name == QLatin1String("thickness_inner_surface") ||
        name.contains(QLatin1String("thickness")) ||
        name.contains(QLatin1String("inner_surface"))) {
        return QStringLiteral("thickness_inner_surface");
    }
    if (name == QLatin1String("length_volume") ||
        name.contains(QLatin1String("length_volume")) ||
        name.contains(QLatin1String("outer_length"))) {
        return QStringLiteral("length_volume");
    }
    if (name == QLatin1String("self_check") || name.contains(QLatin1String("self_check"))) {
        return QStringLiteral("self_check");
    }
    return {};
}

PathDeviceCaptureConfig ConfigManager::resolvePathDeviceCapture(
    const ScanPathConfig& path,
    ScanDeviceKind device)
{
    if (path.capture.configured) {
        return device == ScanDeviceKind::Telescopic ? path.capture.telescopic
                                                    : path.capture.arm;
    }

    // JSON 未写 capture 时按算法回退，避免焊缝误开 CXP、厚度误开智能相机。
    const QString algo = resolvePathAlgorithm(path);
    PathDeviceCaptureConfig cfg;
    if (algo == QLatin1String("weld_section")) {
        cfg.mechEye3d = true;
        cfg.hikSmartC = true;
        cfg.hikCxp = false;
        return cfg;
    }
    if (algo == QLatin1String("thickness_inner_surface")) {
        if (device == ScanDeviceKind::Telescopic) {
            cfg.mechEye3d = false;
            cfg.hikSmartC = false;
            cfg.hikCxp = false;
            return cfg;
        }
        cfg.mechEye3d = true;
        cfg.hikSmartC = false;
        cfg.hikCxp = true;
        return cfg;
    }
    if (algo == QLatin1String("code_read")) {
        if (device == ScanDeviceKind::Telescopic) {
            cfg.mechEye3d = false;
            cfg.hikSmartC = false;
            cfg.hikCxp = false;
            return cfg;
        }
        cfg.mechEye3d = false;
        cfg.hikSmartC = true;
        cfg.hikCxp = false;
        return cfg;
    }
    if (algo == QLatin1String("length_volume")) {
        if (device == ScanDeviceKind::Telescopic) {
            cfg.mechEye3d = false;
            cfg.hikSmartC = false;
            cfg.hikCxp = false;
            return cfg;
        }
        cfg.mechEye3d = true;
        cfg.hikSmartC = false;
        cfg.hikCxp = true;
        return cfg;
    }
    if (algo == QLatin1String("self_check")) {
        if (device == ScanDeviceKind::Telescopic) {
            cfg.mechEye3d = false;
            cfg.hikSmartC = false;
            cfg.hikCxp = false;
            return cfg;
        }
        cfg.mechEye3d = true;
        cfg.hikSmartC = false;
        cfg.hikCxp = true;
        return cfg;
    }

    // 未知算法：保持旧管道默认（臂：3D+智能+CXP；伸缩：3D+智能）
    cfg.mechEye3d = true;
    cfg.hikSmartC = true;
    cfg.hikCxp = (device == ScanDeviceKind::Arm);
    return cfg;
}

PathDeviceCaptureConfig ConfigManager::activeDeviceCapture(ScanDeviceKind device) const
{
    if (const ScanPathConfig* path = activeScanPath()) {
        return resolvePathDeviceCapture(*path, device);
    }
    PathDeviceCaptureConfig cfg;
    cfg.mechEye3d = true;
    cfg.hikSmartC = true;
    cfg.hikCxp = (device == ScanDeviceKind::Arm);
    return cfg;
}

QString ConfigManager::activePathAlgorithm() const
{
    if (const ScanPathConfig* path = activeScanPath()) {
        return resolvePathAlgorithm(*path);
    }
    return {};
}

const ScanPointConfig* ConfigManager::findScanPointByIndex(int segmentIndex) const
{
    if (const ScanPathConfig* active = activeScanPath()) {
        for (const auto& point : active->points) {
            if (point.pointIndex == segmentIndex) {
                return &point;
            }
        }
        // 活跃路径无显式点表时，不跨路径回退（配额模式）
        if (!active->points.empty() || m_scanPathsConfig.activePathId > 0) {
            return nullptr;
        }
    }

    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (!path.enabled) {
            continue;
        }
        for (const auto& point : path.points) {
            if (point.pointIndex == segmentIndex) {
                return &point;
            }
        }
    }
    return nullptr;
}

QString ConfigManager::pointPurpose(int pointIndex) const
{
    if (const ScanPointConfig* point = findScanPointByIndex(pointIndex)) {
        return point->purpose;
    }
    return {};
}

QString ConfigManager::segmentKindForPointIndex(int segmentIndex) const
{
    if (const ScanPathConfig* active = activeScanPath()) {
        for (const auto& point : active->points) {
            if (point.pointIndex == segmentIndex) {
                return active->segmentKind;
            }
        }
        if (m_scanPathsConfig.activePathId > 0) {
            return active->segmentKind;
        }
    }

    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (!path.enabled) {
            continue;
        }
        for (const auto& point : path.points) {
            if (point.pointIndex == segmentIndex) {
                return path.segmentKind;
            }
        }
    }
    return QStringLiteral("external");
}

int ConfigManager::enabledScanPointCount() const
{
    if (m_scanPathsConfig.activePathId > 0) {
        if (const ScanPathConfig* active = findScanPathById(m_scanPathsConfig.activePathId)) {
            const int fromQuota = active->armPointCount + active->telescopicPointCount;
            if (fromQuota > 0) {
                return fromQuota;
            }
            return static_cast<int>(active->points.size());
        }
        return 0;
    }

    int total = 0;
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (!path.enabled) {
            continue;
        }
        const int fromQuota = path.armPointCount + path.telescopicPointCount;
        if (fromQuota > 0) {
            total += fromQuota;
        } else {
            total += static_cast<int>(path.points.size());
        }
    }
    return total;
}

int ConfigManager::enabledArmPointCount() const
{
    if (m_scanPathsConfig.activePathId > 0) {
        if (const ScanPathConfig* active = findScanPathById(m_scanPathsConfig.activePathId)) {
            return active->armPointCount;
        }
        return 0;
    }

    int total = 0;
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (path.enabled) {
            total += path.armPointCount;
        }
    }
    return total;
}

int ConfigManager::enabledTelescopicPointCount() const
{
    if (m_scanPathsConfig.activePathId > 0) {
        if (const ScanPathConfig* active = findScanPathById(m_scanPathsConfig.activePathId)) {
            return active->telescopicPointCount;
        }
        return 0;
    }

    int total = 0;
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (path.enabled) {
            total += path.telescopicPointCount;
        }
    }
    return total;
}

bool ConfigManager::isValidDeviceLocalIndex(ScanDeviceKind device, int localIndex) const
{
    if (localIndex <= 0) {
        return false;
    }
    const int expected = (device == ScanDeviceKind::Telescopic)
                             ? enabledTelescopicPointCount()
                             : enabledArmPointCount();
    if (expected > 0) {
        return localIndex <= expected;
    }
    // 旧版仅 points[]：任一段号命中即合法（无法区分设备）
    return findScanPointByIndex(localIndex) != nullptr;
}

QString ConfigManager::scanDeviceKindToString(ScanDeviceKind device)
{
    switch (device) {
    case ScanDeviceKind::Telescopic:
        return QStringLiteral("telescopic");
    case ScanDeviceKind::Arm:
    default:
        return QStringLiteral("arm");
    }
}

bool ConfigManager::parseScanDeviceKind(const QString& text, ScanDeviceKind* out)
{
    const QString normalized = text.trimmed().toLower();
    ScanDeviceKind kind = ScanDeviceKind::Arm;
    if (normalized == QLatin1String("telescopic") ||
        normalized == QLatin1String("internal") ||
        normalized.startsWith(QLatin1String("internal_"))) {
        kind = ScanDeviceKind::Telescopic;
    } else if (normalized == QLatin1String("arm") ||
               normalized == QLatin1String("external") ||
               normalized.isEmpty()) {
        kind = ScanDeviceKind::Arm;
    } else {
        return false;
    }
    if (out != nullptr) {
        *out = kind;
    }
    return true;
}

/**
 * @brief 写入 config.ini 各节的出厂默认值
 *
 * 在 config.ini 不存在或为空时由 load() 调用；写入后立即 sync() 落盘。
 * 默认值与 load() 中的 fallback 值保持一致，便于首次部署直接运行。
 */
void ConfigManager::writeDefaults(QSettings& settings)
{
    settings.beginGroup("App");
    settings.setValue("version", "0.1.0");
    settings.setValue("environment", "production");
    settings.endGroup();

    settings.beginGroup("Logger");
    settings.setValue("level", 0);
    settings.setValue("rotateDays", 0);  // 保留项，不触发日志删除/覆盖
    settings.endGroup();

    settings.beginGroup("Modbus");
    settings.setValue("host", "127.0.0.1");
    settings.setValue("port", 502);
    settings.setValue("unitId", 3);
    settings.setValue("timeoutMs", 1000);
    settings.setValue("reconnectIntervalMs", 2000);
    settings.endGroup();

    settings.beginGroup("Camera");
    settings.setValue("defaultCamera", "Mech-Eye Nano");
    settings.setValue("scanTimeoutMs", 5000);
    settings.endGroup();

    settings.beginGroup("Station");
    settings.setValue("stationId", 3);
    settings.setValue("stationName", QStringLiteral("第三工位"));
    settings.setValue("scanPathsConfigPath", QStringLiteral("config/scan_paths/station3_placeholder.json"));
    settings.setValue("defaultWorkMode", QStringLiteral("MODE_STATION3"));
    settings.setValue("profileIni", QStringLiteral("config/station_profiles/station3_default.ini"));
    settings.endGroup();

    settings.beginGroup("Vision");
    settings.setValue("mechEyeCameraKey", "Mech-Eye Nano");
    settings.setValue("mechCaptureTimeoutMs", 5000);
    settings.setValue("hikConnectTimeoutMs", 3000);
    settings.setValue("hikCaptureTimeoutMs", 1000);
    settings.setValue("hikExposureTimeUs", 50000);
    settings.setValue("hikGain", 0.0);
    settings.setValue("hikSdkRoot", "third_party/MVS");
    settings.setValue("hikCameraAName", "hik_camera_a");
    settings.setValue("hikCameraAKey", "192.168.10.12");
    settings.setValue("hikCameraAIp", "192.168.10.12");
    settings.setValue("hikCameraASerial", "");
    settings.setValue("hikCameraBName", "hik_camera_b");
    settings.setValue("hikCameraBKey", "192.168.10.13");
    settings.setValue("hikCameraBIp", "192.168.10.13");
    settings.setValue("hikCameraBSerial", "");
    settings.setValue("hikCameraCName", "hik_camera_c");
    settings.setValue("hikCameraCKey", "192.168.8.100");
    settings.setValue("hikCameraCIp", "192.168.8.100");
    settings.setValue("hikCameraCSerial", "");
    settings.setValue("hikCameraCAccessMode", "monitor");
    settings.setValue("hikCameraCTcpListenIp", "192.168.8.13");
    settings.setValue("hikCameraCTcpListenPort", 8999);
    settings.setValue("hikCameraCFtpDirectory", "D:/HikCameraFTP");
    settings.setValue("mechEyeTelescopicKey", "192.168.8.203");
    settings.setValue("hikCameraCTelescopicIp", "192.168.8.204");
    settings.setValue("hikCameraCTelescopicFtpDirectory", "D:/HikCameraFTP/telescopic");
    settings.setValue("mechEyeArmKey", "192.168.8.208");
    settings.setValue("hikCameraCArmIp", "192.168.8.209");
    settings.setValue("hikCameraCArmFtpDirectory", "D:/HikCameraFTP/arm");
    settings.setValue("hikCxpEnabled", true);
    settings.setValue("hikCxpBypassOk", false);
    settings.setValue("hikCxpCaptureTimeoutMs", 5000);
    settings.setValue("hikCxpExposureTimeUs", 50000);
    settings.setValue("hikCxpGain", 0.0);
    settings.setValue("hikCxpSmokeOutputDir", "D:/CxpSmokeTest");
    settings.setValue("hikCxpCameraAName", "ch250_a");
    settings.setValue("hikCxpCameraAKey", "DA9122997");
    settings.setValue("hikCxpCameraASerial", "DA9122997");
    settings.setValue("hikCxpCameraBName", "ch250_b");
    settings.setValue("hikCxpCameraBKey", "DA9122998");
    settings.setValue("hikCxpCameraBSerial", "DA9122998");
    settings.endGroup();

    settings.beginGroup("FlowControl");
    settings.setValue("pollIntervalMs", 100);
    settings.setValue("heartbeatIntervalMs", 1000);
    settings.setValue("simulatedProcessingMs", 300);
    settings.setValue("scanFailurePolicy", QStringLiteral("segment"));
    settings.setValue("algorithmEnabled", true);
    settings.endGroup();

    settings.beginGroup("Tracking");
    settings.setValue("scanSegmentTotal", 3);
    settings.endGroup();

    settings.beginGroup("LbPose");
    settings.setValue("trackConfigFile", QStringLiteral("third_party/LB/track_config.ini"));
    settings.setValue("templateFile", QStringLiteral("third_party/LB/S2_Scanner_Template_20260717.txt"));
    settings.setValue("dataRoot", QStringLiteral("data/LB"));
    settings.setValue("angleToleranceDeg", 2.0);
    settings.setValue("lengthTolerance", 0.5);
    settings.setValue("minPercent", 0.5);
    settings.endGroup();

    settings.beginGroup("OrbbecGemini");
    settings.setValue("orbbecGeminiEnabled", false);
    settings.setValue("orbbecGeminiSdkRoot", QStringLiteral("C:/Program Files/OrbbecSDK 2.8.6"));
    settings.setValue("orbbecGeminiSerial", QString());
    settings.setValue("orbbecGeminiDeviceIndex", 0);
    settings.setValue("orbbecGeminiDepthWidth", 640);
    settings.setValue("orbbecGeminiDepthHeight", 480);
    settings.setValue("orbbecGeminiFps", 15);
    settings.setValue("orbbecGeminiCaptureTimeoutMs", 5000);
    settings.setValue("orbbecGeminiWarmupFrameCount", 5);
    settings.setValue("orbbecGeminiSaveCaptureToDisk", true);
    settings.setValue("orbbecGeminiCaptureCacheDir", QString());
    settings.setValue("orbbecGeminiEnableColorStream", false);
    settings.setValue("orbbecGeminiCaptureOnStart", false);
    settings.endGroup();

    settings.beginGroup("LivoxMid360");
    settings.setValue("livoxMid360Enabled", false);
    settings.setValue(
        "livoxMid360SdkRoot",
        QStringLiteral("third_party/Livox-SDK2"));
    settings.setValue("livoxMid360ConfigFile", QStringLiteral("bin/mid360_config.json"));
    settings.setValue("livoxMid360Serial", QStringLiteral("47MCNCN0035510"));
    settings.setValue("livoxMid360DiscoveryTimeoutMs", 10000);
    settings.endGroup();

    settings.beginGroup("TfminiPlus");
    settings.setValue("tfminiPlusEnabled", false);
    settings.setValue("tfminiPlusPort", QString());
    settings.setValue("tfminiPlusPort2", QString());
    settings.setValue("tfminiPlusBaudRate", 115200);
    settings.setValue("collisionThresholdMm", 0);
    settings.setValue("tfminiPlusLogFrames", false);
    settings.endGroup();

    settings.beginGroup("Hmi");
    settings.setValue("enabled", true);
    settings.setValue("tcpPort", 9900);
    settings.endGroup();

    settings.sync();
    qInfo(LOG_CONFIG) << "已在" << settings.fileName() << "生成默认 config.ini";
}

/**
 * @brief 加载并合并工位 profile
 *
 * 合并优先级（Stage 1）：
 * 1. config.ini [Station] 作为基础
 * 2. profileIni 指向的外部 INI 中 [Station] 同名字段覆盖基础值
 * 3. 结果写入 m_stationProfile，本阶段为只读配置
 */
void ConfigManager::loadStationProfile(QSettings& settings, const QString& configFilePath)
{
    StationProfile profile;
    QString profileIni;

    settings.beginGroup(QStringLiteral("Station"));
    applyStationSettings(settings, profile, &profileIni);
    settings.endGroup();

    const QString resolvedProfileIni = resolveConfigRelativePath(profileIni, configFilePath);
    if (!resolvedProfileIni.isEmpty() && QFileInfo::exists(resolvedProfileIni)) {
        QSettings profileSettings(resolvedProfileIni, QSettings::IniFormat);
        profileSettings.beginGroup(QStringLiteral("Station"));
        applyStationSettings(profileSettings, profile, nullptr);
        profileSettings.endGroup();
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("[Station] 已合并 profileIni=") << resolvedProfileIni;
    } else if (!profileIni.isEmpty()) {
        qWarning(LOG_CONFIG).noquote()
            << QStringLiteral("[Station] profileIni 不存在，忽略：")
            << resolvedProfileIni;
    }

    m_stationProfile = profile;
    qInfo(LOG_CONFIG).noquote()
        << QStringLiteral("[Station] stationId=") << stationIdToInt(m_stationProfile.stationId)
        << QStringLiteral(" name=") << m_stationProfile.stationName
        << QStringLiteral(" scanPaths=") << (m_stationProfile.scanPathsConfigPath.isEmpty()
                                                ? QStringLiteral("<fallback scan_paths_config.json>")
                                                : m_stationProfile.scanPathsConfigPath)
        << QStringLiteral(" workMode=") << workModeIdToString(m_stationProfile.defaultWorkMode);
}

/**
 * @brief 从 config.ini 加载全部 INI 配置到成员结构体
 *
 * 按节顺序读取：App → Logger → Modbus → Camera → Station（profile）→
 * Vision → FlowControl → Tracking → OrbbecGemini → LivoxMid360 → TfminiPlus → Hmi。
 * 读取 Logger.level 后同步设置 Logger 单例的最低输出级别。
 */
void ConfigManager::load(const QString& filePath)
{
    const QFileInfo fileInfo(filePath);
    const bool fileExists = fileInfo.exists() && fileInfo.size() > 0;
    m_configFilePath = QDir::cleanPath(filePath);

    QSettings settings(filePath, QSettings::IniFormat);
    if (!fileExists) {
        qWarning(LOG_CONFIG) << "config.ini 未找到或为空。正在生成默认配置...";
        writeDefaults(settings);
    }

    // --- [App] ---
    settings.beginGroup("App");
    m_appConfig.version = settings.value("version", "0.1.0").toString();
    m_appConfig.environment = settings.value("environment", "production").toString();
    settings.endGroup();

    // --- [Logger] ---
    settings.beginGroup("Logger");
    m_loggerConfig.level = settings.value("level", 0).toInt();
    m_loggerConfig.rotateDays = settings.value("rotateDays", 7).toInt();
    settings.endGroup();

    // --- [Modbus] ---
    settings.beginGroup("Modbus");
    m_modbusConfig.host = settings.value("host", "127.0.0.1").toString();
    m_modbusConfig.port = settings.value("port", 502).toInt();
    m_modbusConfig.unitId = settings.value("unitId", 1).toInt();
    m_modbusConfig.timeoutMs = settings.value("timeoutMs", 1000).toInt();
    m_modbusConfig.reconnectIntervalMs = settings.value("reconnectIntervalMs", 2000).toInt();
    settings.endGroup();

    // --- [Camera] ---
    settings.beginGroup("Camera");
    m_cameraConfig.defaultCamera = settings.value("defaultCamera", "Mech-Eye Nano").toString();
    m_cameraConfig.scanTimeoutMs = settings.value("scanTimeoutMs", 5000).toInt();
    settings.endGroup();

    // --- [Station] + profileIni 合并 ---
    loadStationProfile(settings, filePath);

    // --- [Vision] ---
    settings.beginGroup("Vision");
    m_visionConfig.mechEyeCameraKey = settings.value("mechEyeCameraKey", m_cameraConfig.defaultCamera).toString();
    m_visionConfig.mechCaptureTimeoutMs = settings.value("mechCaptureTimeoutMs", m_cameraConfig.scanTimeoutMs).toInt();
    m_visionConfig.mechDepthRangeMin = settings.value("mechDepthRangeMin", 100).toInt();
    m_visionConfig.mechDepthRangeMax = settings.value("mechDepthRangeMax", 2000).toInt();
    m_visionConfig.hikConnectTimeoutMs = settings.value("hikConnectTimeoutMs", 3000).toInt();
    m_visionConfig.hikCaptureTimeoutMs = settings.value("hikCaptureTimeoutMs", 1000).toInt();
    m_visionConfig.hikExposureTimeUs =
        static_cast<float>(settings.value("hikExposureTimeUs", 50000).toDouble());
    m_visionConfig.hikGain = static_cast<float>(settings.value("hikGain", 0.0).toDouble());
    m_visionConfig.hikSdkRoot = settings.value("hikSdkRoot", "third_party/MVS").toString();
    m_visionConfig.hikCameraA.logicalName = settings.value("hikCameraAName", "hik_camera_a").toString();
    m_visionConfig.hikCameraA.cameraKey = settings.value("hikCameraAKey", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.ipAddress = settings.value("hikCameraAIp", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.serialNumber = settings.value("hikCameraASerial", "").toString();
    m_visionConfig.hikCameraB.logicalName = settings.value("hikCameraBName", "hik_camera_b").toString();
    m_visionConfig.hikCameraB.cameraKey = settings.value("hikCameraBKey", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.ipAddress = settings.value("hikCameraBIp", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.serialNumber = settings.value("hikCameraBSerial", "").toString();
    m_visionConfig.hikCameraC.logicalName = settings.value("hikCameraCName", "hik_camera_c").toString();
    m_visionConfig.hikCameraC.cameraKey = settings.value("hikCameraCKey", "192.168.8.100").toString();
    m_visionConfig.hikCameraC.ipAddress = settings.value("hikCameraCIp", "192.168.8.100").toString();
    m_visionConfig.hikCameraC.serialNumber = settings.value("hikCameraCSerial", "").toString();
    m_visionConfig.hikCameraC.accessMode = settings.value("hikCameraCAccessMode", "monitor").toString();
    m_visionConfig.hikCameraCTcpListenIp = settings.value("hikCameraCTcpListenIp", "192.168.8.13").toString();
    m_visionConfig.hikCameraCTcpListenPort = static_cast<quint16>(settings.value("hikCameraCTcpListenPort", 8999).toUInt());
    m_visionConfig.hikCameraCFtpDirectory = settings.value("hikCameraCFtpDirectory", "D:/HikCameraFTP").toString();
    m_visionConfig.hikCxpEnabled = settings.value("hikCxpEnabled", false).toBool();
    m_visionConfig.hikCxpCaptureTimeoutMs = settings.value("hikCxpCaptureTimeoutMs", 5000).toInt();
    m_visionConfig.hikCxpExposureTimeUs =
        static_cast<float>(settings.value("hikCxpExposureTimeUs", 50000).toDouble());
    m_visionConfig.hikCxpGain = static_cast<float>(settings.value("hikCxpGain", 0.0).toDouble());
    m_visionConfig.hikCxpSmokeOutputDir =
        settings.value("hikCxpSmokeOutputDir", "D:/CxpSmokeTest").toString();
    m_visionConfig.hikCxpCameraA.logicalName =
        settings.value("hikCxpCameraAName", "ch250_a").toString();
    m_visionConfig.hikCxpCameraA.cameraKey =
        settings.value("hikCxpCameraAKey", "DA9122997").toString();
    m_visionConfig.hikCxpCameraA.serialNumber =
        settings.value("hikCxpCameraASerial", "DA9122997").toString();
    m_visionConfig.hikCxpCameraB.logicalName =
        settings.value("hikCxpCameraBName", "ch250_b").toString();
    m_visionConfig.hikCxpCameraB.cameraKey =
        settings.value("hikCxpCameraBKey", "DA9122998").toString();
    m_visionConfig.hikCxpCameraB.serialNumber =
        settings.value("hikCxpCameraBSerial", "DA9122998").toString();
    m_visionConfig.hikCxpBypassOk = settings.value("hikCxpBypassOk", false).toBool();

    const QString telescopicMechKey =
        settings.value("mechEyeTelescopicKey", m_visionConfig.mechEyeCameraKey).toString();
    const QString telescopicHikIp =
        settings.value("hikCameraCTelescopicIp", m_visionConfig.hikCameraC.ipAddress).toString();
    const QString telescopicFtpDir = settings.value(
        "hikCameraCTelescopicFtpDirectory",
        m_visionConfig.hikCameraCFtpDirectory.isEmpty()
            ? QStringLiteral("D:/HikCameraFTP/telescopic")
            : m_visionConfig.hikCameraCFtpDirectory)
                                         .toString();
    m_visionConfig.telescopicGroup.mechEye.logicalName =
        QStringLiteral("mech_eye_telescopic");
    m_visionConfig.telescopicGroup.mechEye.cameraKey = telescopicMechKey;
    m_visionConfig.telescopicGroup.mechEye.ipAddress = telescopicMechKey;
    m_visionConfig.telescopicGroup.hikCameraC.logicalName =
        QStringLiteral("hik_camera_c_telescopic");
    m_visionConfig.telescopicGroup.hikCameraC.cameraKey = telescopicHikIp;
    m_visionConfig.telescopicGroup.hikCameraC.ipAddress = telescopicHikIp;
    m_visionConfig.telescopicGroup.hikCameraCFtpDirectory = telescopicFtpDir;

    const QString armMechKey =
        settings.value("mechEyeArmKey", QStringLiteral("192.168.8.208")).toString();
    const QString armHikIp =
        settings.value("hikCameraCArmIp", QStringLiteral("192.168.8.209")).toString();
    const QString armFtpDir = settings
                                  .value("hikCameraCArmFtpDirectory", QStringLiteral("D:/HikCameraFTP/arm"))
                                  .toString();
    m_visionConfig.armGroup.mechEye.logicalName = QStringLiteral("mech_eye_arm");
    m_visionConfig.armGroup.mechEye.cameraKey = armMechKey;
    m_visionConfig.armGroup.mechEye.ipAddress = armMechKey;
    m_visionConfig.armGroup.hikCameraC.logicalName = QStringLiteral("hik_camera_c_arm");
    m_visionConfig.armGroup.hikCameraC.cameraKey = armHikIp;
    m_visionConfig.armGroup.hikCameraC.ipAddress = armHikIp;
    m_visionConfig.armGroup.hikCameraCFtpDirectory = armFtpDir;

    settings.endGroup();

    // --- [FlowControl] ---
    settings.beginGroup("FlowControl");
    m_flowControlConfig.pollIntervalMs = settings.value("pollIntervalMs", 100).toInt();
    m_flowControlConfig.heartbeatIntervalMs = settings.value("heartbeatIntervalMs", 1000).toInt();
    m_flowControlConfig.simulatedProcessingMs = settings.value("simulatedProcessingMs", 300).toInt();
    m_flowControlConfig.scanFailurePolicy =
        settings.value("scanFailurePolicy", QStringLiteral("segment")).toString().trimmed().toLower();
    m_flowControlConfig.algorithmEnabled = settings.value("algorithmEnabled", true).toBool();
    settings.endGroup();

    // 兼容工位1配置键：若存在 [Resume] scanFailurePolicy 则覆盖（便于双工位共用 ini）
    settings.beginGroup("Resume");
    if (settings.contains(QStringLiteral("scanFailurePolicy"))) {
        m_flowControlConfig.scanFailurePolicy =
            settings.value("scanFailurePolicy").toString().trimmed().toLower();
    }
    settings.endGroup();

    if (m_flowControlConfig.scanFailurePolicy != QStringLiteral("path")
        && m_flowControlConfig.scanFailurePolicy != QStringLiteral("workpiece")) {
        m_flowControlConfig.scanFailurePolicy = QStringLiteral("segment");
    }
    qInfo(LOG_CONFIG).noquote()
        << QStringLiteral("扫描失败清理策略 scanFailurePolicy=")
        << m_flowControlConfig.scanFailurePolicy
        << QStringLiteral(" algorithmEnabled=") << m_flowControlConfig.algorithmEnabled;

    // --- [Tracking] ---
    settings.beginGroup("Tracking");
    m_trackingConfig.scanSegmentTotal = settings.value("scanSegmentTotal", 3).toInt();
    settings.endGroup();


    settings.beginGroup("LbPose");
    m_lbPoseConfig.trackConfigFile = resolveConfigRelativePath(
        settings.value(
                    "trackConfigFile",
                    QStringLiteral("third_party/LB/track_config.ini"))
            .toString(),
        m_configFilePath);
    m_lbPoseConfig.dataRoot = resolveConfigRelativePath(
        settings.value("dataRoot", QStringLiteral("data/LB")).toString(),
        m_configFilePath);
    m_lbPoseConfig.leftPattern = settings.value("leftPattern", "").toString();
    m_lbPoseConfig.rightPattern = settings.value("rightPattern", "").toString();
    m_lbPoseConfig.templateFile = resolveConfigRelativePath(
        settings.value(
                    "templateFile",
                    QStringLiteral("third_party/LB/S2_Scanner_Template_20260717.txt"))
            .toString(),
        m_configFilePath);
    const float legacyCosTolerance = settings.value("cosTolerance", 0.015).toFloat();
    m_lbPoseConfig.angleToleranceDeg =
        settings.value("angleToleranceDeg", legacyCosTolerance).toFloat();
    m_lbPoseConfig.lengthTolerance = settings.value("lengthTolerance", 0.5).toFloat();
    m_lbPoseConfig.minPercent = settings.value("minPercent", 0.5).toFloat();
    settings.endGroup();

    // --- [OrbbecGemini] ---
    settings.beginGroup("OrbbecGemini");
    m_orbbecGeminiConfig.enabled = settings.value("orbbecGeminiEnabled", false).toBool();
    m_orbbecGeminiConfig.sdkRoot = settings.value(
        "orbbecGeminiSdkRoot",
        QStringLiteral("C:/Program Files/OrbbecSDK 2.8.6")).toString();
    m_orbbecGeminiConfig.serial = settings.value("orbbecGeminiSerial", QString()).toString();
    m_orbbecGeminiConfig.deviceIndex = settings.value("orbbecGeminiDeviceIndex", 0).toInt();
    m_orbbecGeminiConfig.depthWidth = settings.value("orbbecGeminiDepthWidth", 640).toInt();
    m_orbbecGeminiConfig.depthHeight = settings.value("orbbecGeminiDepthHeight", 480).toInt();
    m_orbbecGeminiConfig.fps = settings.value("orbbecGeminiFps", 15).toInt();
    m_orbbecGeminiConfig.captureTimeoutMs =
        settings.value("orbbecGeminiCaptureTimeoutMs", 5000).toInt();
    m_orbbecGeminiConfig.warmupFrameCount =
        settings.value("orbbecGeminiWarmupFrameCount", 5).toInt();
    m_orbbecGeminiConfig.saveCaptureToDisk =
        settings.value("orbbecGeminiSaveCaptureToDisk", true).toBool();
    m_orbbecGeminiConfig.captureCacheDir =
        settings.value("orbbecGeminiCaptureCacheDir", QString()).toString().trimmed();
    m_orbbecGeminiConfig.enableColorStream =
        settings.value("orbbecGeminiEnableColorStream", false).toBool();
    m_orbbecGeminiConfig.captureOnStart =
        settings.value("orbbecGeminiCaptureOnStart", false).toBool();
    settings.endGroup();

    // --- [LivoxMid360] ---
    settings.beginGroup("LivoxMid360");
    m_livoxMid360Config.enabled = settings.value("livoxMid360Enabled", false).toBool();
    m_livoxMid360Config.sdkRoot = settings.value(
        "livoxMid360SdkRoot",
        QStringLiteral("third_party/Livox-SDK2")).toString();
    m_livoxMid360Config.configFile = settings.value(
        "livoxMid360ConfigFile",
        QStringLiteral("bin/mid360_config.json")).toString();
    m_livoxMid360Config.serial = settings.value("livoxMid360Serial", QString()).toString();
    m_livoxMid360Config.discoveryTimeoutMs =
        settings.value("livoxMid360DiscoveryTimeoutMs", 10000).toInt();
    settings.endGroup();

    // --- [TfminiPlus] ---
    settings.beginGroup("TfminiPlus");
    m_tfminiPlusConfig.enabled = settings.value("tfminiPlusEnabled", false).toBool();
    m_tfminiPlusConfig.portName = settings.value("tfminiPlusPort", QString()).toString().trimmed();
    m_tfminiPlusConfig.portName2 =
        settings.value("tfminiPlusPort2", QString()).toString().trimmed();
    m_tfminiPlusConfig.baudRate = settings.value("tfminiPlusBaudRate", 115200).toInt();
    m_tfminiPlusConfig.collisionThresholdMm =
        settings.value("collisionThresholdMm", 0).toInt();
    m_tfminiPlusConfig.logFrames =
        settings.value("tfminiPlusLogFrames", false).toBool();
    settings.endGroup();

    // --- [Hmi] ---
    settings.beginGroup("Hmi");
    m_hmiConfig.enabled = settings.value("enabled", true).toBool();
    {
        const int port = settings.value("tcpPort", 9900).toInt();
        m_hmiConfig.tcpPort = static_cast<quint16>(
            qBound(1, port, 65535));
    }
    settings.endGroup();

    // 将 Logger.level 映射为 Qt 消息级别并应用到 Logger 单例
    QtMsgType minType = QtDebugMsg;
    switch (m_loggerConfig.level) {
        case 0: minType = QtDebugMsg; break;
        case 1: minType = QtInfoMsg; break;
        case 2: minType = QtWarningMsg; break;
        case 3: minType = QtCriticalMsg; break;
        default: minType = QtDebugMsg; break;
    }

    if (Logger* logger = Logger::instance()) {
        logger->setMinLevel(minType);
    }

    qInfo(LOG_CONFIG) << "已从以下位置加载 config.ini：" << filePath;
    qInfo(LOG_CONFIG).noquote()
        << QStringLiteral("Modbus 配置：")
        << QStringLiteral(" host=") << m_modbusConfig.host
        << QStringLiteral(" port=") << m_modbusConfig.port
        << QStringLiteral(" unitId=") << m_modbusConfig.unitId
        << QStringLiteral(" timeoutMs=") << m_modbusConfig.timeoutMs
        << QStringLiteral(" reconnectIntervalMs=") << m_modbusConfig.reconnectIntervalMs;
}

/**
 * @brief 加载扫描路径配置（JSON 格式）
 *
 * 推荐格式（双设备配额 + 活跃路径）：
 * - activePathId：当前运行路径（方案 A）
 * - devices[]：{ device: "arm"|"telescopic", totalPoints: N }
 * - 或扁平字段 armPointCount / telescopicPointCount
 * - points[].purpose：可选，供后续算法路由
 *
 * 兼容旧格式：
 * - points[] + totalPoints + 可选 segmentKind
 */
void ConfigManager::loadScanPathsConfig(const QString& jsonFilePath)
{
    if (!QFileInfo::exists(jsonFilePath)) {
        qWarning(LOG_CONFIG) << "扫描路径配置文件不存在：" << jsonFilePath;
        qWarning(LOG_CONFIG) << "将使用空配置，多路径扫描功能不可用。";
        return;
    }

    QFile file(jsonFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qCritical(LOG_CONFIG) << "无法打开扫描路径配置文件：" << jsonFilePath;
        return;
    }

    const QByteArray jsonData = file.readAll();
    file.close();

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(jsonData, &parseError);

    if (parseError.error != QJsonParseError::NoError) {
        qCritical(LOG_CONFIG) << "扫描路径配置文件 JSON 解析失败："
                              << parseError.errorString()
                              << "位置：" << parseError.offset;
        return;
    }

    if (!doc.isObject()) {
        qCritical(LOG_CONFIG) << "扫描路径配置文件格式错误：根节点不是对象";
        return;
    }

    const QJsonObject root = doc.object();

    m_scanPathsConfig.version = root.value("version").toString("1.0");
    m_scanPathsConfig.lastModified = root.value("lastModified").toString();
    m_scanPathsConfig.activePathId = root.value("activePathId").toInt(0);

    const QJsonArray pathsArray = root.value("scanPaths").toArray();
    m_scanPathsConfig.scanPaths.clear();
    m_scanPathsConfig.scanPaths.reserve(pathsArray.size());

    for (const QJsonValue& pathValue : pathsArray) {
        const QJsonObject pathObj = pathValue.toObject();

        ScanPathConfig pathConfig;
        pathConfig.pathId = pathObj.value("pathId").toInt();
        pathConfig.enabled = pathObj.value("enabled").toBool(true);
        pathConfig.name = pathObj.value("name").toString();
        pathConfig.algorithm = pathObj.value("algorithm").toString();
        pathConfig.segmentKind = pathObj.value("segmentKind").toString(QStringLiteral("external"));
        pathConfig.description = pathObj.value("description").toString();
        pathConfig.totalPoints = pathObj.value("totalPoints").toInt();
        pathConfig.armPointCount = pathObj.value("armPointCount").toInt(0);
        pathConfig.telescopicPointCount = pathObj.value("telescopicPointCount").toInt(0);
        pathConfig.volumeRadiusMm = pathObj.value("volumeRadiusMm").toDouble(0.0);

        const QJsonObject captureObj = pathObj.value("capture").toObject();
        if (!captureObj.isEmpty()) {
            pathConfig.capture.configured = true;
            const auto parseDeviceCapture = [](const QJsonObject& obj,
                                               PathDeviceCaptureConfig* out) {
                if (obj.isEmpty() || out == nullptr) {
                    return;
                }
                if (obj.contains(QLatin1String("mechEye3d"))) {
                    out->mechEye3d = obj.value(QLatin1String("mechEye3d")).toBool(out->mechEye3d);
                }
                if (obj.contains(QLatin1String("hikSmartC"))) {
                    out->hikSmartC = obj.value(QLatin1String("hikSmartC")).toBool(out->hikSmartC);
                }
                if (obj.contains(QLatin1String("hikCxp"))) {
                    out->hikCxp = obj.value(QLatin1String("hikCxp")).toBool(out->hikCxp);
                }
            };
            // 未写字段时：臂默认 3D+智能、CXP 关；伸缩默认 3D+智能（与焊缝矩阵一致，避免漏配误开 CXP）
            pathConfig.capture.arm = PathDeviceCaptureConfig{};
            pathConfig.capture.arm.mechEye3d = true;
            pathConfig.capture.arm.hikSmartC = true;
            pathConfig.capture.arm.hikCxp = false;
            pathConfig.capture.telescopic = PathDeviceCaptureConfig{};
            pathConfig.capture.telescopic.mechEye3d = true;
            pathConfig.capture.telescopic.hikSmartC = true;
            pathConfig.capture.telescopic.hikCxp = false;
            parseDeviceCapture(captureObj.value("arm").toObject(), &pathConfig.capture.arm);
            parseDeviceCapture(
                captureObj.value("telescopic").toObject(), &pathConfig.capture.telescopic);
        }

        const QJsonArray devicesArray = pathObj.value("devices").toArray();
        pathConfig.devices.clear();
        pathConfig.devices.reserve(devicesArray.size());
        for (const QJsonValue& deviceValue : devicesArray) {
            const QJsonObject deviceObj = deviceValue.toObject();
            ScanDeviceKind kind = ScanDeviceKind::Arm;
            if (!parseScanDeviceKind(deviceObj.value("device").toString(), &kind)) {
                qWarning(LOG_CONFIG).noquote()
                    << QStringLiteral("路径 %1 含未知 device，已忽略：")
                           .arg(pathConfig.pathId)
                    << deviceObj.value("device").toString();
                continue;
            }
            ScanDeviceQuota quota;
            quota.device = kind;
            quota.totalPoints = deviceObj.value("totalPoints").toInt(0);
            pathConfig.devices.push_back(quota);
            if (kind == ScanDeviceKind::Telescopic) {
                pathConfig.telescopicPointCount = quota.totalPoints;
            } else {
                pathConfig.armPointCount = quota.totalPoints;
            }
        }

        const QJsonArray pointsArray = pathObj.value("points").toArray();
        pathConfig.points.clear();
        pathConfig.points.reserve(pointsArray.size());

        for (const QJsonValue& pointValue : pointsArray) {
            const QJsonObject pointObj = pointValue.toObject();

            ScanPointConfig pointConfig;
            pointConfig.pointIndex = pointObj.value("pointIndex").toInt();
            pointConfig.needRotation = pointObj.value("needRotation").toBool(false);
            pointConfig.purpose = pointObj.value("purpose").toString();

            pathConfig.points.push_back(pointConfig);
        }

        // 旧版仅 points[]：按 segmentKind 归入对应设备配额
        if (pathConfig.armPointCount == 0 && pathConfig.telescopicPointCount == 0 &&
            !pathConfig.points.empty()) {
            ScanDeviceKind kind = ScanDeviceKind::Arm;
            parseScanDeviceKind(pathConfig.segmentKind, &kind);
            const int count = static_cast<int>(pathConfig.points.size());
            if (kind == ScanDeviceKind::Telescopic) {
                pathConfig.telescopicPointCount = count;
            } else {
                pathConfig.armPointCount = count;
            }
            if (pathConfig.devices.empty()) {
                ScanDeviceQuota quota;
                quota.device = kind;
                quota.totalPoints = count;
                pathConfig.devices.push_back(quota);
            }
        }

        // 新版仅配额、无 points：补齐 devices 列表与 totalPoints
        if (pathConfig.devices.empty()) {
            if (pathConfig.armPointCount > 0) {
                pathConfig.devices.push_back(
                    ScanDeviceQuota{ScanDeviceKind::Arm, pathConfig.armPointCount});
            }
            if (pathConfig.telescopicPointCount > 0) {
                pathConfig.devices.push_back(
                    ScanDeviceQuota{ScanDeviceKind::Telescopic, pathConfig.telescopicPointCount});
            }
        }
        const int quotaSum = pathConfig.armPointCount + pathConfig.telescopicPointCount;
        if (quotaSum > 0) {
            pathConfig.totalPoints = quotaSum;
        }

        m_scanPathsConfig.scanPaths.push_back(pathConfig);
    }

    QString validationError;
    if (!validateScanPathsConfig(&validationError)) {
        qWarning(LOG_CONFIG) << "扫描路径配置验证失败：" << validationError;
    }

    qInfo(LOG_CONFIG) << "已从以下位置加载扫描路径配置：" << jsonFilePath;
    qInfo(LOG_CONFIG).noquote()
        << "扫描路径配置："
        << "版本=" << m_scanPathsConfig.version
        << "activePathId=" << m_scanPathsConfig.activePathId
        << "路径数=" << m_scanPathsConfig.scanPaths.size()
        << "活跃配额 臂=" << enabledArmPointCount()
        << "伸缩杆=" << enabledTelescopicPointCount()
        << "合计=" << enabledScanPointCount();

    for (const auto& path : m_scanPathsConfig.scanPaths) {
        const bool isActive = (m_scanPathsConfig.activePathId > 0)
                                  ? (path.pathId == m_scanPathsConfig.activePathId)
                                  : path.enabled;
        const PathDeviceCaptureConfig armCap =
            resolvePathDeviceCapture(path, ScanDeviceKind::Arm);
        const PathDeviceCaptureConfig telCap =
            resolvePathDeviceCapture(path, ScanDeviceKind::Telescopic);
        qInfo(LOG_CONFIG).noquote()
            << "  路径" << path.pathId
            << (path.name.isEmpty() ? QString() : QStringLiteral(" name=") + path.name)
            << (resolvePathAlgorithm(path).isEmpty()
                    ? QString()
                    : QStringLiteral(" algorithm=") + resolvePathAlgorithm(path))
            << "启用=" << path.enabled
            << "活跃=" << isActive
            << "机械臂点数=" << path.armPointCount
            << "伸缩杆点数=" << path.telescopicPointCount
            << "总点数=" << path.totalPoints
            << "显式点表=" << path.points.size()
            << QStringLiteral(" capture源=")
            << (path.capture.configured ? QStringLiteral("json") : QStringLiteral("algorithm"))
            << QStringLiteral(" arm[3D=%1 smart=%2 cxp=%3]")
                   .arg(armCap.mechEye3d)
                   .arg(armCap.hikSmartC)
                   .arg(armCap.hikCxp)
            << QStringLiteral(" telescopic[3D=%1 smart=%2]")
                   .arg(telCap.mechEye3d)
                   .arg(telCap.hikSmartC);
    }
}

/**
 * @brief 验证扫描路径配置的合法性
 *
 * 规则：
 * - 所有路径的 pathId 互不重复
 * - 若存在 points[]：数量须与 totalPoints 一致（旧版），且启用路径内 pointIndex 合法
 * - 新版设备配额：arm/telescopic 点数均 >= 0，且至少一侧 > 0（启用路径）
 * - activePathId > 0 时必须能在 scanPaths 中找到对应 pathId
 */
bool ConfigManager::validateScanPathsConfig(QString* errorMessage) const
{
    std::vector<int> pathIds;
    pathIds.reserve(m_scanPathsConfig.scanPaths.size());

    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (std::find(pathIds.begin(), pathIds.end(), path.pathId) != pathIds.end()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 ID 重复：%1").arg(path.pathId);
            }
            return false;
        }
        pathIds.push_back(path.pathId);

        if (path.armPointCount < 0 || path.telescopicPointCount < 0) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 %1 设备配额不能为负").arg(path.pathId);
            }
            return false;
        }

        if (!path.points.empty() &&
            static_cast<int>(path.points.size()) != path.totalPoints &&
            path.armPointCount + path.telescopicPointCount == 0) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 %1 的点位数量不匹配：配置 %2，实际 %3")
                                    .arg(path.pathId)
                                    .arg(path.totalPoints)
                                    .arg(path.points.size());
            }
            return false;
        }

        // 有 activePathId 时，非活跃路径允许暂不配点数；活跃路径必须有配额
        const bool mustHaveQuota = (m_scanPathsConfig.activePathId > 0)
                                       ? (path.pathId == m_scanPathsConfig.activePathId)
                                       : path.enabled;
        if (mustHaveQuota && path.armPointCount == 0 && path.telescopicPointCount == 0 &&
            path.points.empty()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 %1 未配置任何设备点数").arg(path.pathId);
            }
            return false;
        }

        if (!path.points.empty() &&
            path.armPointCount + path.telescopicPointCount == static_cast<int>(path.points.size())) {
            // 显式点表：校验 pointIndex（活跃或 enabled 路径）
            if (!mustHaveQuota && m_scanPathsConfig.activePathId > 0) {
                continue;
            }
            std::vector<int> seen;
            for (const auto& point : path.points) {
                if (point.pointIndex <= 0) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral("路径 %1 的点位索引无效：%2")
                                            .arg(path.pathId)
                                            .arg(point.pointIndex);
                    }
                    return false;
                }
                if (std::find(seen.begin(), seen.end(), point.pointIndex) != seen.end()) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral("路径 %1 段号重复：%2")
                                            .arg(path.pathId)
                                            .arg(point.pointIndex);
                    }
                    return false;
                }
                seen.push_back(point.pointIndex);
            }
        }
    }

    if (m_scanPathsConfig.activePathId > 0 &&
        findScanPathById(m_scanPathsConfig.activePathId) == nullptr) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("activePathId=%1 在 scanPaths 中不存在")
                                .arg(m_scanPathsConfig.activePathId);
        }
        return false;
    }

    return true;
}

}  // namespace common
}  // namespace scan_tracking
