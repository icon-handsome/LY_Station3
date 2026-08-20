#pragma once

// Orbbec Gemini 深度相机模块的公共类型定义。
// 包含运行时状态、错误码、设备配置、采集请求/结果等数据结构，
// 并通过 Q_DECLARE_METATYPE 注册到 Qt 元对象系统，支持跨线程信号槽传递。

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

namespace scan_tracking {
namespace orbbec_gemini {

/* Orbbec Gemini 服务/工作线程的运行时状态 */
enum class OrbbecGeminiRuntimeState {
    Idle = 0,
    Enumerating,
    Opening,
    Ready,
    Capturing,
    Failed,
    Stopped,
};

/* 单次深度采集的错误码 */
enum class OrbbecCaptureErrorCode {
    Success = 0,
    NotStarted = 1,
    NotReady = 2,
    Busy = 3,
    Timeout = 4,
    CaptureFailed = 5,
    SaveFailed = 6,
    InvalidRequest = 7,
    UnknownError = 8,
};

/* 打开 Orbbec 设备时的配置参数 */
struct OrbbecGeminiOpenConfig {
    QString serial;              // 设备序列号；非空时优先按序列号打开，忽略 deviceIndex
    int deviceIndex = 0;         // 设备列表索引，serial 为空时使用
    int depthWidth = 640;        // 深度流宽度（像素）
    int depthHeight = 480;       // 深度流高度（像素）
    int fps = 15;                // 深度流帧率
    int captureTimeoutMs = 5000; // 默认采集超时（毫秒）
    int warmupFrameCount = 5;    // 启动深度流后丢弃的预热帧数，用于稳定曝光/深度
    bool saveCaptureToDisk = true;   // 是否将采集结果保存到磁盘
    QString captureCacheRoot;    // 采集缓存根目录；为空时使用全局默认路径
    bool enableColorStream = false;  // 是否同时启用彩色流（当前采集流程主要使用深度流）
};

/* 枚举到的 Orbbec 设备摘要信息 */
struct OrbbecGeminiDeviceSummary {
    int index = -1;              // 在设备列表中的索引
    QString name;                // 设备型号名称
    QString serialNumber;        // 序列号
    QString firmwareVersion;     // 固件版本
    QString connectionType;      // 连接类型（如 USB）
    quint16 pid = 0;             // USB 产品 ID
    quint16 vid = 0;             // USB 厂商 ID
    QString uid;                 // SDK 内部唯一标识
};

/* 单次深度采集请求 */
struct OrbbecCaptureRequest {
    quint64 requestId = 0;   // 请求 ID，由 OrbbecGeminiService 分配，用于关联结果
    int timeoutMs = 5000;    // 等待深度帧的超时（毫秒）；0 表示使用服务默认值
    bool saveToDisk = true;  // 是否保存深度 PNG 与点云 PLY；可与全局配置叠加判断
};

/* 单次深度采集结果 */
struct OrbbecCaptureResult {
    quint64 requestId = 0;
    OrbbecCaptureErrorCode errorCode = OrbbecCaptureErrorCode::Success;
    QString errorMessage;        // 失败时的可读错误描述
    OrbbecGeminiDeviceSummary deviceInfo;  // 当前已打开设备的信息
    int depthWidth = 0;          // 深度图宽度
    int depthHeight = 0;         // 深度图高度
    float depthValueScale = 0.0f;    // 深度原始值到毫米的缩放系数（来自 SDK）
    int validDepthPixelCount = 0;    // 非零深度像素数量
    int pointCloudPointCount = 0;    // 点云点数（含无效点，保存 PLY 时会过滤）
    qint64 captureDurationMs = 0;    // 本次采集耗时（毫秒）
    QString depthRawPngPath;     // 16 位原始深度 PNG 路径
    QString depthPreviewPngPath; // 8 位归一化预览深度 PNG 路径
    QString pointCloudPlyPath;   // ASCII PLY 点云文件路径
};

}  // namespace orbbec_gemini
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecCaptureResult)
Q_DECLARE_METATYPE(QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary>)
