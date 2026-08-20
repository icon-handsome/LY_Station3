#pragma once

// 将跨线程传递的数据类型集中定义在这里，避免 service 和 worker 相互包含。
// 这些类型后续也可能被 flow_control 或 tracking 复用，因此命名尽量保持业务语义清晰。

#include <memory>
#include <vector>

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

namespace scan_tracking {
namespace mech_eye {

/* 采集模式 */
enum class CaptureMode {
    Capture3DOnly = 0,
    Capture2DAnd3D = 1,
    Capture2DOnly = 2,
};

/* 采集与连接过程中的统一错误码 */
enum class CaptureErrorCode {
    Success = 0,
    NotStarted = 1,
    NotConnected = 2,
    Busy = 3,
    DiscoverFailed = 4,
    ConnectFailed = 5,
    CaptureFailed = 6,
    DisconnectFailed = 7,
    Timeout = 8,
    InvalidRequest = 9,
    UnknownError = 10,
    /// Mech-Eye SDK 原生崩溃（如 AV）；进程内 SDK 已隔离，需重启进程后再用
    SdkNativeFault = 11,
};

/* 相机运行状态 */
enum class CameraRuntimeState {
    Idle = 0,
    Discovering = 1,
    Connecting = 2,
    Ready = 3,
    Capturing = 4,
    Disconnecting = 5,
    Error = 6,
    Stopped = 7,
};

/* 相机信息快照，用于跨线程传递和状态展示 */
struct CameraInfoSnapshot {
    QString model;  // 相机型号
    QString serialNumber;   // 序列号
    QString ipAddress;  // IP 地址
    QString firmwareVersion;    // 固件版本
    bool connected = false; // 是否已连接
};

/* 与 3D 纹理点云对齐的 2D 灰度图（Capture2DAnd3D 时有效） */
struct GrayTextureFrame {
    std::shared_ptr<std::vector<uint8_t>> pixels;
    int width = 0;
    int height = 0;

    bool isValid() const
    {
        return pixels && !pixels->empty() && width > 0 && height > 0 &&
               static_cast<int>(pixels->size()) >= width * height;
    }
};

/* 点云帧数据
 * 使用 shared_ptr 保存大数组，避免跨线程传递时发生深拷贝。
 */
struct PointCloudFrame {
    std::shared_ptr<std::vector<float>> pointsXYZ;  // 每三个 float 表示一个点的 x, y, z 坐标
    std::shared_ptr<std::vector<float>> normalsXYZ;  // 每三个 float 表示一个点的法向量 x, y, z 分量
    int width = 0;  // 点云的宽度
    int height = 0; // 点云的高度
    int pointCount = 0; // 点云中的点数量
    quint64 frameId = 0;  // 唯一标识符
    qint64 timestampMs = 0; // 采集时间戳，单位毫秒

    /* 判断点云是否有效 */
    bool isValid() const
    {
        return pointsXYZ && !pointsXYZ->empty() && pointCount > 0;
    }

    /* 判断是否带有法向量数据 */
    bool hasNormals() const
    {
        return normalsXYZ && static_cast<int>(normalsXYZ->size()) == pointCount * 3;
    }

    /* 获取法向量数量 */
    int normalCount() const
    {
        return normalsXYZ ? static_cast<int>(normalsXYZ->size() / 3) : 0;
    }
};

/* 采集请求参数 */
struct CaptureRequest {
    quint64 requestId = 0;  // 唯一标识符
    QString cameraKey;  // 相机唯一标识
    CaptureMode mode = CaptureMode::Capture3DOnly;  // 采集模式
    int timeoutMs = 30000;  // 采集超时时间，单位毫秒，默认30秒
};

/* 采集结果 */
struct CaptureResult {
    quint64 requestId = 0;  // 唯一标识符，与 CaptureRequest 中的 requestId 一致
    QString cameraKey;  // 相机唯一标识
    CaptureMode mode = CaptureMode::Capture3DOnly;  // 采集模式
    CaptureErrorCode errorCode = CaptureErrorCode::Success; // 错误码，Success 表示采集成功
    QString errorMessage;   // 错误描述，便于日志记录和调试
    CameraInfoSnapshot cameraInfo;  // 采集时的相机信息快照
    PointCloudFrame pointCloud; // 工作点云：LB 成功后为变换云，否则为原始云
    // LB 成功变换时保留原始云（shared_ptr 别名，无深拷贝），供 cloud.ply 落盘；检测仍读 pointCloud。
    PointCloudFrame pointCloudRaw;
    GrayTextureFrame texture2D; // 与点云对齐的 2D 灰度纹理（仅 Capture2DAnd3D）
    qint64 elapsedMs = 0;   // 采集耗时，单位毫秒

    /* 判断采集是否成功 */
    bool success() const
    {
        return errorCode == CaptureErrorCode::Success;
    }
};

/// 进程内 Mech-Eye SDK 是否已因原生 AV 隔离（需重启进程后再用）
bool isMechEyeSdkProcessIsolated();

}  // namespace mech_eye
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureMode)    
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureErrorCode)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CameraRuntimeState)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CameraInfoSnapshot)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::GrayTextureFrame)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::PointCloudFrame)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureResult)
