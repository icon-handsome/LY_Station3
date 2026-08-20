#pragma once

#include <QSettings>
#include <QString>
#include <QVector>
#include <QtCore/QtGlobal>
#include <vector>

#include "scan_tracking/common/station_profile.h"

namespace scan_tracking {
namespace common {

/// 应用级元信息，对应 config.ini [App] 节。
struct AppConfig {
    QString version;      ///< 应用版本号，用于日志与 HMI 展示
    QString environment;  ///< 运行环境标识，如 production / development
};

/// 日志系统配置，对应 config.ini [Logger] 节。
struct LoggerConfig {
    int level;       ///< 最低日志级别：0=Debug, 1=Info, 2=Warning, 3=Critical
    int rotateDays;  ///< 日志保留天数（保留项，当前不触发自动删除）
};

/// Modbus TCP 连接参数，对应 config.ini [Modbus] 节。
struct ModbusConfig {
    QString host;              ///< PLC / 网关 IP 地址
    int port;                  ///< Modbus TCP 端口，默认 502
    int unitId;                ///< 从站单元号
    int timeoutMs;             ///< 单次读写超时（毫秒）
    int reconnectIntervalMs;   ///< 断线后重连间隔（毫秒）
};

/// 通用相机配置（Mech-Eye 等），对应 config.ini [Camera] 节。
struct CameraConfig {
    QString defaultCamera;  ///< 默认相机标识 / 设备名
    int scanTimeoutMs;      ///< 单次扫描超时（毫秒）
};

/// 单台海康相机的连接端点描述（逻辑名、Key、IP、序列号等）。
struct VisionCameraEndpointConfig {
    QString logicalName;   ///< 业务逻辑名称，如 hik_camera_a
    QString cameraKey;       ///< SDK 枚举 / 连接 Key（常为 IP 或序列号）
    QString ipAddress;       ///< 相机 IP
    QString serialNumber;    ///< 相机序列号（可选，用于精确匹配）
    QString accessMode = QStringLiteral("exclusive");  ///< 访问模式：exclusive / monitor 等
};

/// 单组视觉设备：Mech-Eye + 海康智能 C 及 C 的 FTP 落盘目录。
struct VisionDeviceGroupConfig {
    VisionCameraEndpointConfig mechEye;
    VisionCameraEndpointConfig hikCameraC;
    QString hikCameraCFtpDirectory;
};

/// 视觉子系统配置，对应 config.ini [Vision] 节。
/// 涵盖 Mech-Eye 深度相机、海康 GigE 相机 A/B/C、CXP 相机及 Camera C 的 TCP/FTP 监听。
struct VisionConfig {
    QString mechEyeCameraKey;   ///< Mech-Eye 设备 Key（遗留字段，缺省映射伸缩杆组）
    int mechCaptureTimeoutMs;   ///< Mech-Eye 采集超时
    int mechDepthRangeMin;      ///< Mech-Eye 深度范围下限（毫米）
    int mechDepthRangeMax;      ///< Mech-Eye 深度范围上限（毫米）
    int hikConnectTimeoutMs;    ///< 海康相机连接超时
    int hikCaptureTimeoutMs;    ///< 海康 GigE 采集超时
    float hikExposureTimeUs = 50000.0f;  ///< 海康曝光时间（微秒）
    float hikGain = 0.0f;                ///< 海康增益
    QString hikSdkRoot;                  ///< 海康 MVS SDK 根目录
    VisionCameraEndpointConfig hikCameraA;
    VisionCameraEndpointConfig hikCameraB;
    VisionCameraEndpointConfig hikCameraC;
    bool hikCxpEnabled = false;              ///< 是否启用 CXP 相机链路
    int hikCxpCaptureTimeoutMs = 5000;
    float hikCxpExposureTimeUs = 50000.0f;
    float hikCxpGain = 0.0f;
    QString hikCxpSmokeOutputDir;              ///< CXP 冒烟测试输出目录
    VisionCameraEndpointConfig hikCxpCameraA;
    VisionCameraEndpointConfig hikCxpCameraB;
    QString hikCameraCTcpListenIp;             ///< Camera C 触发用 TCP 监听 IP
    quint16 hikCameraCTcpListenPort;           ///< Camera C 触发用 TCP 端口
    QString hikCameraCFtpDirectory;            ///< Camera C FTP 落盘目录（遗留，缺省映射伸缩杆组）
    VisionDeviceGroupConfig telescopicGroup;   ///< 伸缩杆组：Mech + 海康 C
    VisionDeviceGroupConfig armGroup;          ///< 机械臂组：Mech + 海康 C
    /// true：段扫/自检仍正常进入；仅不采 CXP、不因 CXP 失败阻断（CXP 另作他用时开）
    /// 放在结构体末尾，避免插入中部导致 VisionConfig ABI/偏移错位引发启动崩溃。
    bool hikCxpBypassOk = false;
};

/// 流程控制轮询与心跳参数，对应 config.ini [FlowControl] 节。
struct FlowControlConfig {
    int pollIntervalMs;           ///< 状态机主循环轮询间隔
    int heartbeatIntervalMs;      ///< Modbus 心跳写入间隔
    int simulatedProcessingMs;    ///< 模拟处理耗时（调试 / 占位）
    /// 扫描失败清理策略（对齐工位1 [Resume] scanFailurePolicy；本工位暂无完整续跑）
    /// segment=仅剔失败本段；path=清当前路径段缓存（保留 run_*）；workpiece=整表清零并解绑 run_*
    QString scanFailurePolicy = QStringLiteral("segment");
    /// false：跳过所有路径算法解算（厚度/容积/焊缝等），检测通道直接返回 OK，仅跑采集主流程。
    bool algorithmEnabled = true;
};

/// 扫描跟踪相关参数，对应 config.ini [Tracking] 节。
struct TrackingConfig {
    int scanSegmentTotal = 3;  ///< 扫描段总数 fallback（无 scan_paths JSON 时使用）
};

/// LB 光学双目标记点位姿检测配置，对应 config.ini [LbPose] 节。
struct LbPoseConfig {
    /// LB 算法权威配置（标定、GeoHash、Recon 等），对应 third_party/LB/track_config.ini
    QString trackConfigFile;
    /// 可选：覆盖 track_config.ini [Paths] template_points
    QString templateFile;
    /// 离线/回退：数据根目录与左右图 glob（在线 CXP 采集不使用）
    QString dataRoot;
    QString leftPattern;
    QString rightPattern;
    /// LB 查询角度容差（度）
    float angleToleranceDeg = 2.0f;
    /// LB 查询长度容差（mm）
    float lengthTolerance = 0.5f;
    /// LB 最低票数占比
    float minPercent = 0.5f;
};

/// Orbbec Gemini 深度相机配置，对应 config.ini [OrbbecGemini] 节。
struct OrbbecGeminiConfig {
    bool enabled = false;
    QString sdkRoot;
    QString serial;
    int deviceIndex = 0;
    int depthWidth = 640;
    int depthHeight = 480;
    int fps = 15;
    int captureTimeoutMs = 5000;
    int warmupFrameCount = 5;       ///< 启动后丢弃的预热帧数
    bool saveCaptureToDisk = true;
    QString captureCacheDir;        ///< 为空时使用默认缓存根目录
    bool enableColorStream = false;
    bool captureOnStart = true;     ///< 服务启动时是否立即采集一帧
};

/// Livox Mid-360 激光雷达配置，对应 config.ini [LivoxMid360] 节。
struct LivoxMid360Config {
    bool enabled = false;
    QString sdkRoot;
    QString configFile;             ///< Livox SDK 设备配置文件路径
    QString serial;
    int discoveryTimeoutMs = 10000;
};

/// TFmini Plus 测距 / 碰撞监测配置，对应 config.ini [TfminiPlus] 节。
/// 支持最多两路串口（现场双 TF，各自 USB 转串口对应不同 COM）。
struct TfminiPlusConfig {
    bool enabled = false;
    QString portName;               ///< TF1 串口名，如 COM3
    QString portName2;              ///< TF2 串口名，如 COM4；空则仅开一路
    int baudRate = 115200;
    int collisionThresholdMm = 0;   ///< 碰撞阈值（毫米），0 表示禁用
    bool logFrames = false;         ///< 是否逐帧打印测距日志
};

/// HMI TCP 服务配置，对应 config.ini [Hmi] 节。
struct HmiConfig {
    bool enabled = true;
    quint16 tcpPort = 9900;       ///< HMI 客户端连接端口
};

/// 扫描设备角色（机械臂 / 伸缩杆），与 PLC 触发器一一对应。
enum class ScanDeviceKind {
    Arm = 0,         ///< Trig_ScanSegment → 机械臂相机组
    Telescopic = 1,  ///< Trig_TelescopicScan → 伸缩杆相机组
};

/// 扫描路径中的单个点位定义（旧版 points[]；新版可省略，改用设备配额）。
struct ScanPointConfig {
    int pointIndex = 0;           ///< 设备内本地段号（1..N）；臂读 AO47/40015，伸缩杆读 AO48/40016
    bool needRotation = false;    ///< true → 预留彩色/2D 扩展；Orbbec 主流程采集深度+点云
    QString purpose;              ///< 点位用途：thickness / inner_surface 等；空表示未指定
};

/// 一条扫描路径下某设备的配额（二维：设备 × 本地索引上限）。
struct ScanDeviceQuota {
    ScanDeviceKind device = ScanDeviceKind::Arm;
    int totalPoints = 0;  ///< 该设备需要采集的点数（本地索引 1..totalPoints）
};

/// 某设备组在本路径上的相机启用矩阵（来自 scan_paths JSON 的 capture.arm / capture.telescopic）。
struct PathDeviceCaptureConfig {
    bool mechEye3d = true;   ///< 梅卡 3D（本轮管道仍始终采 Mech；供后续按路径关闭）
    bool hikSmartC = true;   ///< 海康智能相机 C
    bool hikCxp = false;     ///< 海康 CXP 双目（仅机械臂有意义；伸缩杆忽略）
};

/// 路径级采集相机开关；configured=false 表示 JSON 未写 capture，运行时按 algorithm 回退。
struct PathCaptureConfig {
    bool configured = false;
    PathDeviceCaptureConfig arm;
    PathDeviceCaptureConfig telescopic;
};

/// 一条扫描路径的定义（来自 scan_paths JSON 的 scanPaths[] 元素）。
struct ScanPathConfig {
    int pathId = 0;               ///< 路径 ID，在 JSON 内唯一
    bool enabled = true;          ///< 无 activePathId 时参与配额汇总；有 activePathId 时仅作文档/回退
    QString name;                 ///< 稳定短名，由工位定义
    QString algorithm;            ///< 检测算法标识，由工位算法模块定义
    QString segmentKind = QStringLiteral("external");  ///< 旧字段；新版以 devices 为准
    QString description;          ///< 人类可读描述
    int totalPoints = 0;          ///< 声明总点数；新版 = 各设备配额之和
    int armPointCount = 0;        ///< 机械臂配额（可与 devices 互写）
    int telescopicPointCount = 0; ///< 伸缩杆配额
    double volumeRadiusMm = 0.0;  ///< 保留扩展字段，由具体算法自行解释
    PathCaptureConfig capture;    ///< 路径级相机启用矩阵
    std::vector<ScanDeviceQuota> devices;  ///< 设备配额列表（推荐配置方式）
    std::vector<ScanPointConfig> points;   ///< 显式点表（可带 purpose）；新版配额模式下可空
};

/// 扫描路径 JSON 文件的根结构（由工位 profile 的 scanPathsConfigPath 指定）。
struct ScanPathsConfig {
    std::vector<ScanPathConfig> scanPaths;
    int activePathId = 0;   ///< >0 时运行时配额/校验仅使用该 pathId；可由 setActivePathId 热切换
    QString version;        ///< JSON 文件版本
    QString lastModified;   ///< 最后修改时间（字符串，来自 JSON 元数据）
};

/**
 * @brief 全局配置管理器（单例）
 *
 * 负责在进程启动时加载并持有全部运行时配置，是各模块获取 config.ini 与 scan_paths JSON
 * 参数的统一入口。配置来源分为两层：
 *
 * 1. **主配置文件 config.ini**（INI 格式，QSettings 读写）
 *    - 各功能节：App、Logger、Modbus、Camera、Vision、FlowControl 等
 *    - [Station] 节定义工位 profile，并可引用外部 profileIni 做字段覆盖
 *
 * 2. **扫描路径 JSON**（路径由 StationProfile::scanPathsConfigPath 决定）
 *    - 描述多路径扫描的 pathId、点位索引、是否需旋转等
 *    - 加载后可通过 findScanPointByIndex / enabledScanPointCount 查询
 *
 * **生命周期**：须在 main() 中先调用 initialize()，进程退出前调用 cleanup()。
 * 典型用法：`ConfigManager::instance()->modbusConfig()`。
 *
 * @note 单例非线程安全创建；initialize() 应在主线程、任何业务模块启动前调用一次。
 */
class ConfigManager {
public:
    /// 创建单例并加载 config.ini 与 scan_paths JSON。重复调用无副作用。
    static void initialize();

    /// 销毁单例并释放内存。重复调用无副作用。
    static void cleanup();

    /// 返回单例指针；若尚未 initialize() 则返回 nullptr 并输出警告。
    static ConfigManager* instance();

    const AppConfig& appConfig() const;
    const LoggerConfig& loggerConfig() const;
    const ModbusConfig& modbusConfig() const;
    const CameraConfig& cameraConfig() const;
    const VisionConfig& visionConfig() const;
    const FlowControlConfig& flowControlConfig() const;
    const TrackingConfig& trackingConfig() const;
    const LbPoseConfig& lbPoseConfig() const;
    const OrbbecGeminiConfig& orbbecGeminiConfig() const;
    const LivoxMid360Config& livoxMid360Config() const;
    const TfminiPlusConfig& tfminiPlusConfig() const;

    /// 已加载的主 config.ini 绝对路径。
    QString configFilePath() const;

    const HmiConfig& hmiConfig() const;
    const ScanPathsConfig& scanPathsConfig() const;

    /// 合并后的工位 profile（含 feature 开关与 scan_paths 文件路径）。
    const StationProfile& stationProfile() const;

    /// 按 pathId 查找路径；未找到返回 nullptr。
    const ScanPathConfig* findScanPathById(int pathId) const;

    /// 当前活跃路径：优先 JSON activePathId；未配置或找不到时回退第一条 enabled 路径。
    const ScanPathConfig* activeScanPath() const;

    /// 活跃路径 pathId；无可用路径时返回 0。
    int activePathId() const;

    /// 运行时切换活跃路径（不改 JSON 文件）。pathId 须存在于 scanPaths；成功返回 true。
    bool setActivePathId(int pathId);

    /// 按 JSON 顺序切到下一条 enabled 路径（到末尾则回到第一条）。返回新 pathId；无可切路径返回 0。
    int advanceToNextEnabledPath();

    /// 按 JSON 顺序收集全部 enabled 路径的 pathId。
    QVector<int> enabledPathIds() const;

    /// 活跃路径 name；无则空串。
    QString activePathName() const;

    /// 活跃路径算法标识（已归一化）；无则空串。
    QString activePathAlgorithm() const;

    /// 将 path.algorithm / path.name 归一为算法标识。
    static QString resolvePathAlgorithm(const ScanPathConfig& path);

    /// 解析路径在指定设备组上的相机开关：优先 JSON capture，否则按 algorithm 回退。
    static PathDeviceCaptureConfig resolvePathDeviceCapture(
        const ScanPathConfig& path,
        ScanDeviceKind device);

    /// 活跃路径在指定设备组上的相机开关；无活跃路径时返回旧管道默认（臂：3D+智能+CXP）。
    PathDeviceCaptureConfig activeDeviceCapture(ScanDeviceKind device) const;

    /// 按全局段号（pointIndex）在活跃路径（或已启用路径）中查找点位；未找到返回 nullptr。
    /// @note 新版双设备配额模式下，优先用 isValidDeviceLocalIndex。
    const ScanPointConfig* findScanPointByIndex(int segmentIndex) const;

    /// 活跃路径中某本地段号的 purpose；未配置返回空串。
    QString pointPurpose(int pointIndex) const;

    /// 段号所属路径的 segmentKind；未找到时返回 "external"。
    QString segmentKindForPointIndex(int segmentIndex) const;

    /// 活跃路径点位总数（有 activePathId 时）；否则为所有 enabled 路径配额之和。
    int enabledScanPointCount() const;

    /// 活跃路径机械臂应采集点数（无 activePathId 时汇总 enabled）。
    int enabledArmPointCount() const;

    /// 活跃路径伸缩杆应采集点数（无 activePathId 时汇总 enabled）。
    int enabledTelescopicPointCount() const;

    /// 校验某设备本地段号是否在（活跃路径）配额范围内（1..totalPoints）。
    bool isValidDeviceLocalIndex(ScanDeviceKind device, int localIndex) const;

    /// 设备枚举 ↔ JSON / 日志标签。
    static QString scanDeviceKindToString(ScanDeviceKind device);
    static bool parseScanDeviceKind(const QString& text, ScanDeviceKind* out);

private:
    ConfigManager();
    ~ConfigManager();

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    /// 从指定路径读取 config.ini；文件缺失时自动生成默认值。
    void load(const QString& filePath);

    /// 解析 [Station] 并可选合并 profileIni 中的同名字段。
    void loadStationProfile(QSettings& settings, const QString& configFilePath);

    /// 向 QSettings 写入各节的出厂默认值（首次运行或 config.ini 缺失时）。
    void writeDefaults(QSettings& settings);

    /// 从 JSON 文件加载 scanPaths 配置并执行校验。
    void loadScanPathsConfig(const QString& jsonFilePath);

    /// 校验路径 ID 唯一、点位数量一致、全局段号不重复等规则。
    bool validateScanPathsConfig(QString* errorMessage = nullptr) const;

    static ConfigManager* s_instance;

    AppConfig m_appConfig;
    LoggerConfig m_loggerConfig;
    ModbusConfig m_modbusConfig;
    CameraConfig m_cameraConfig;
    VisionConfig m_visionConfig;
    FlowControlConfig m_flowControlConfig;
    TrackingConfig m_trackingConfig;
    LbPoseConfig m_lbPoseConfig;
    OrbbecGeminiConfig m_orbbecGeminiConfig;
    LivoxMid360Config m_livoxMid360Config;
    TfminiPlusConfig m_tfminiPlusConfig;
    QString m_configFilePath;
    HmiConfig m_hmiConfig;
    StationProfile m_stationProfile;
    ScanPathsConfig m_scanPathsConfig;
};

}  // namespace common
}  // namespace scan_tracking
