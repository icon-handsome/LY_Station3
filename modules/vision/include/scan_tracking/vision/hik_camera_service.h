#pragma once

// 海康 GigE / USB 相机服务（MVS SDK）。
//
// 通过设备枚举与序列号/IP 匹配打开相机，连接与采图在可接合的 std::thread 中执行，
// 避免阻塞主线程和 PLC 轮询。用于非 CXP 场景或 monitor 只读模式（可与 SCMVS 共存）。
// 支持运行时读写曝光、增益等 GenICam 参数。
// stop() 会关闭异步回投闸门并 join 工作线程，避免 detach 悬空 this。

#include <QtCore/QObject>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCameraService : public QObject {
    Q_OBJECT

public:
    /* @param roleName 逻辑角色名，用于日志与 stateChanged 区分多路相机 */
    explicit HikCameraService(const QString& roleName, QObject* parent = nullptr);
    ~HikCameraService() override;

    /* 启动服务并后台异步连接；写入默认曝光/增益 */
    void start(
        const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
        int defaultCaptureTimeoutMs,
        float exposureTimeUs = 50000.0f,
        float gain = 0.0f);

    /* 停止服务并释放设备 */
    void stop();

    bool isStarted() const;
    bool isConnected() const;
    QString roleName() const;
    const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig() const;

    /* 请求一次 Mono 采图，结果经 poseCaptureFinished 异步返回 */
    quint64 requestPoseCapture(const QString& preferredCameraKey = {}, int timeoutMs = 0);

    /* 读取相机当前参数快照（需已连接，线程安全） */
    HikCameraParams readParams(QString* errorMessage = nullptr);

    /* 修改相机参数（需已连接，线程安全）
     * 只修改 params 中 valid==true 的字段；调用前请确保相机未在采图
     */
    bool writeParams(const HikCameraParams& params, QString* errorMessage = nullptr);

signals:
    void poseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void stateChanged(QString roleName, QString stateText, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private:
    class Impl;  ///< PIMPL：MVS 设备句柄、采图缓冲与参数读写实现

    static void registerMetaTypes();
    QString resolveCameraKey(const QString& preferredCameraKey) const;
    bool ensureConnected(const QString& preferredCameraKey, QString* errorMessage);
    bool captureMonoFrame(int timeoutMs, const QString& cameraKey, QString* errorMessage, HikMonoFrame* outFrame = nullptr);
    bool openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage);
    void closeDevice();
    void closeDeviceUnlocked();
    void startAsyncConnect();
    void joinWorkerThreads();

    QString m_roleName;
    scan_tracking::common::VisionCameraEndpointConfig m_endpointConfig;
    int m_defaultCaptureTimeoutMs = 1000;
    float m_exposureTimeUs = 50000.0f;
    float m_gain = 0.0f;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    std::atomic_bool m_connectInFlight = false;
    std::atomic_bool m_captureInFlight = false;
    // 工作线程拷贝 shared_ptr；stop 置 false 后禁止回投主线程，避免悬空 this。
    std::shared_ptr<std::atomic_bool> m_acceptAsyncResults;
    std::mutex m_workerThreadsMutex;
    std::thread m_connectThread;
    std::thread m_captureThread;
    // 串行化 open/close，避免连接线程与采图线程并行开设备踩句柄。
    std::mutex m_deviceLifecycleMutex;
    Impl* m_impl = nullptr;
};

}  // namespace vision
}  // namespace scan_tracking
