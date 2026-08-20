#pragma once

// Mech-Eye 3D 相机服务（主线程门面）。
//
// 在独立 QThread 中托管 MechEyeWorker，所有阻塞式 SDK 调用（发现、连接、采图）
// 均在 worker 线程完成，结果经 Qt 信号异步回传主线程。
// 供 VisionPipelineService / StateMachine 发起分段点云采集。

#include <QtCore/QObject>

#include "scan_tracking/mech_eye/mech_eye_types.h"

QT_BEGIN_NAMESPACE
class QThread;
QT_END_NAMESPACE

namespace scan_tracking {
namespace mech_eye {

class MechEyeWorker;

/// 主线程侧 Mech-Eye 门面：请求排队、busy 互斥、状态转发
class MechEyeService : public QObject {
    Q_OBJECT

public:
    /* 构造函数
     * @param roleName 逻辑角色名（如 "梅卡-伸缩杆" / "梅卡-机械臂"），用于日志与状态区分
     * @param parent Qt 父对象指针
     */
    explicit MechEyeService(const QString& roleName = QString(), QObject* parent = nullptr);

    /* 析构函数，负责安全停止 worker 线程并释放资源 */
    ~MechEyeService() override;

    /* 获取逻辑角色名 */
    QString roleName() const { return m_roleName; }

    /* 启动 Mech-Eye 服务线程、初始化元类型并连接信号槽（从 ConfigManager 读取默认相机 Key） */
    void start();

    /* 启动 Mech-Eye 服务并绑定指定默认相机 Key（双实例场景） */
    void start(const QString& defaultCameraKey);

    /* 停止 Mech-Eye 服务线程并清理当前状态 */
    void stop();

    /* 获取当前是否有采集请求正在执行 */
    bool isBusy() const { return m_busy; }

    /* 获取当前相机运行状态 */
    CameraRuntimeState state() const { return m_currentState; }

    /* 获取相机是否已连接（基于最近一次状态更新）
     * 后续可用于设备在线状态字等场景，当前仅保存不使用。
     */
    bool isCameraConnected() const { return m_cameraConnected; }

    /* 最近一次 fatalError 错误码（无则 Success） */
    CaptureErrorCode lastFatalCode() const { return m_lastFatalCode; }

    /* 最近一次 fatalError 描述 */
    QString lastFatalMessage() const { return m_lastFatalMessage; }

    /* 主动刷新相机在线状态
     * 该调用会转发到 worker 线程，由 worker 重新探测或查询相机状态。
     */
    void requestRefreshStatus();

    /* 发起一次采集请求
     * @param cameraKey 相机标识，可以是型号、序列号、IP 或设备名
     * @param mode 采集模式
     * @param timeoutMs 采集超时时间（毫秒）
     * @return 请求 ID；若当前不可用则返回 0
     */
    quint64 requestCapture(
        const QString& cameraKey,
        CaptureMode mode = CaptureMode::Capture3DOnly,
        int timeoutMs = 30000);

signals:
    /* 采集完成信号
     * @param result 采集结果
     */
    void captureFinished(scan_tracking::mech_eye::CaptureResult result);

    /* 相机运行状态变化信号
     * @param newState 新状态
     * @param description 状态说明
     */
    void stateChanged(scan_tracking::mech_eye::CameraRuntimeState newState, QString description);

    /* 致命错误信号
     * @param code 错误码
     * @param message 错误描述
     */
    void fatalError(scan_tracking::mech_eye::CaptureErrorCode code, QString message);

    /* 投递给 worker 线程的启动请求 */
    void sig_startWorker(QString defaultCameraKey);

    /* 投递给 worker 线程的停止请求 */
    void sig_stopWorker();

    /* 投递给 worker 线程的状态刷新请求 */
    void sig_refreshStatus();

    /* 投递给 worker 线程的采集请求 */
    void sig_performCapture(scan_tracking::mech_eye::CaptureRequest request);

private slots:
    /* 处理 worker 返回的采集完成结果 */
    void onWorkerCaptureFinished(scan_tracking::mech_eye::CaptureResult result);

    /* 处理 worker 返回的状态变化 */
    void onWorkerStateChanged(scan_tracking::mech_eye::CameraRuntimeState newState, QString description);

    /* 处理 worker 返回的致命错误 */
    void onWorkerFatalError(scan_tracking::mech_eye::CaptureErrorCode code, QString message);

private:
    /* 注册跨线程传递所需的 Qt 元类型 */
    static void registerMetaTypes();

    QThread* m_workerThread = nullptr;
    MechEyeWorker* m_worker = nullptr;

    QString m_roleName;
    QString m_defaultCameraKey;
    int m_defaultCaptureTimeoutMs = 5000;
    quint64 m_nextRequestId = 1;
    bool m_busy = false;
    bool m_stopping = false;
    bool m_started = false;
    bool m_cameraConnected = false;  // 相机连接状态（后续使用）
    CameraRuntimeState m_currentState = CameraRuntimeState::Idle;
    CaptureErrorCode m_lastFatalCode = CaptureErrorCode::Success;
    QString m_lastFatalMessage;
};

}  // namespace mech_eye
}  // namespace scan_tracking