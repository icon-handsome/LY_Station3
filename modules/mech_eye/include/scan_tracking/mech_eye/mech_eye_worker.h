#pragma once

// Mech-Eye SDK 工作线程实现。
//
// 由 MechEyeService 所在 QThread 驱动，执行相机发现、连接、2D/3D 采图及
// Frame3D → PointCloudFrame 转换。禁止在主线程直接调用本类槽函数。

#include <QtCore/QObject>
#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace mmind {
namespace eye {
struct CameraInfo;
class Frame3D;
class Frame2DAnd3D;
}  // namespace eye
}  // namespace mmind

namespace scan_tracking {
namespace mech_eye {

/// SDK 操作执行体；与 MechEyeService 通过 QueuedConnection 通信
class MechEyeWorker : public QObject {
    Q_OBJECT

public:
    /* 构造函数
     * @param roleName 逻辑角色名（如 "梅卡-伸缩杆" / "梅卡-机械臂"），用于日志与状态区分
     * @param parent Qt 父对象指针
     */
    explicit MechEyeWorker(const QString& roleName = QString(), QObject* parent = nullptr);

    /* 析构函数，负责在对象销毁前断开相机并释放内部实现 */
    ~MechEyeWorker() override;

public slots:
    /* 启动 worker，并尝试连接默认相机
     * @param defaultCameraKey 默认相机标识
     */
    void startWorker(const QString& defaultCameraKey);

    /* 停止 worker，并安全释放 SDK 资源 */
    void stopWorker();

    /* 刷新相机状态，必要时重新连接 */
    void refreshStatus();

    /* 执行一次采集请求
     * @param request 采集请求参数
     */
    void performCapture(const scan_tracking::mech_eye::CaptureRequest& request);

signals:
    /* 采集完成信号
     * @param result 采集结果
     */
    void captureFinished(scan_tracking::mech_eye::CaptureResult result);

    /* 状态变化信号
     * @param newState 新状态
     * @param description 状态说明
     */
    void stateChanged(scan_tracking::mech_eye::CameraRuntimeState newState, QString description);

    /* 致命错误信号
     * @param code 错误码
     * @param message 错误描述
     */
    void fatalError(scan_tracking::mech_eye::CaptureErrorCode code, QString message);

private:
    class Impl;  ///< PIMPL：mmind::eye::Camera 实例与 SDK 帧缓冲，隔离 Mech-Eye 头文件

    /* 设置运行状态并向外发射通知
     * @param newState 新状态
     * @param description 状态说明
     */
    void setRuntimeState(CameraRuntimeState newState, const QString& description = {});

    /* 为消息加上角色前缀，便于区分双设备组 */
    QString taggedMessage(const QString& message) const;

    /* 将 SDK 错误码映射为项目内部错误码
     * @param sdkErrorCode SDK 返回值
     * @return 项目内部错误码
     */
    CaptureErrorCode mapSdkError(int sdkErrorCode) const;

    /* 将相机信息转换为快照结构
     * @param info SDK 相机信息
     * @param connected 是否认为当前已连接
     * @return 相机快照
     */
    CameraInfoSnapshot makeSnapshot(const mmind::eye::CameraInfo& info, bool connected) const;

    /* 确保指定相机已连接
     * @param cameraKey 相机标识
     * @param timeoutMs 连接超时时间
     * @param errorMessage 错误信息输出参数
     * @return 是否连接成功
     */
    bool ensureConnected(const QString& cameraKey, int timeoutMs, QString* errorMessage);

    /* 主动连接指定相机
     * @param cameraKey 相机标识
     * @param timeoutMs 连接超时时间
     * @param errorMessage 错误信息输出参数
     * @return 是否连接成功
     */
    bool connectCamera(const QString& cameraKey, int timeoutMs, QString* errorMessage);

    /* 断开当前相机连接
     * @param errorMessage 错误信息输出参数
     * @return 是否断开成功
     */
    bool disconnectCamera(QString* errorMessage = nullptr);

    /* SDK 原生崩溃后重建 Camera 实例，避免继续使用损坏对象 */
    void resetSdkCamera();

    /* 构造失败结果
     * @param request 原始请求
     * @param errorCode 错误码
     * @param errorMessage 错误描述
     * @param elapsedMs 已耗时
     * @return 失败结果对象
     */
    CaptureResult makeFailureResult(
        const CaptureRequest& request,
        CaptureErrorCode errorCode,
        const QString& errorMessage,
        qint64 elapsedMs) const;

    /* 将 3D 点云帧转换为内部点云结构 */
    PointCloudFrame buildPointCloud3D(const mmind::eye::Frame3D& frame) const;

    /* 将 2D+3D 点云帧转换为内部点云结构 */
    PointCloudFrame buildPointCloud2DAnd3D(const mmind::eye::Frame2DAnd3D& frame) const;

    /* 连接成功后打印相机基础参数（曝光、增益、分辨率等）；内部走 SEH */
    void printCameraParameters();

    /* 无 SEH 的参数读取实现，仅由 printCameraParameters 经 invokeVoid 调用 */
    void printCameraParametersUnguarded();

    QString m_roleName;
    QString m_defaultCameraKey;
    CameraRuntimeState m_state = CameraRuntimeState::Idle;
    CameraInfoSnapshot m_cameraInfo;
    bool m_busy = false;
    bool m_connected = false;
    Impl* m_impl = nullptr;
};

}  // namespace mech_eye
}  // namespace scan_tracking