#pragma once

// 多相机视觉流水线：Mech-Eye + 海康 CXP 双目 + 海康智能 C（可并行），含 LB 位姿解算。

#include <QtCore/QObject>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCxpCameraService;
class HikCameraCController;

class VisionPipelineService : public QObject {
    Q_OBJECT

public:
    VisionPipelineService(
        scan_tracking::mech_eye::MechEyeService* mechEyeTelescopicService,
        scan_tracking::mech_eye::MechEyeService* mechEyeArmService,
        HikCxpCameraService* hikCameraAService,
        HikCxpCameraService* hikCameraBService,
        HikCameraCController* hikCameraCController = nullptr,
        QObject* parent = nullptr);
    ~VisionPipelineService() override;

    void start(const scan_tracking::common::VisionConfig& config);
    void stop();

    bool isStarted() const { return m_started; }
    VisionPipelineState state() const { return m_state; }

    /// CXP 双目是否均已连接（hikCxpEnabled=false 或未注入服务时视为不需要 / 未连接由调用方解释）。
    bool isHikCxpAConnected() const;
    bool isHikCxpBConnected() const;

    /// 组合采集通道开关（路径级矩阵解析后传入）。
    /// 真实 CXP 采图：与全局 hikCxpEnabled 取与；hikCxpBypassOk=true 时不采 CXP、不因 CXP 失败阻断。
    struct BundleCaptureOptions {
        bool useMechEye = true;    ///< 本轮仍强制需要 Mech 服务；预留按路径关闭
        bool useHikCxp = true;     ///< 是否采 CXP 双目（伸缩杆侧会被忽略）
        bool useHikSmartC = true;  ///< 是否触发海康智能相机 C
    };

    quint64 requestCaptureBundle(
        int segmentIndex,
        quint32 taskId,
        scan_tracking::mech_eye::CaptureMode mechCaptureMode);

    /// @param telescopicConcurrentHikC true：伸缩杆设备组；false：机械臂设备组。
    quint64 requestCaptureBundle(
        int segmentIndex,
        quint32 taskId,
        scan_tracking::mech_eye::CaptureMode mechCaptureMode,
        bool telescopicConcurrentHikC);

    quint64 requestCaptureBundle(
        int segmentIndex,
        quint32 taskId,
        scan_tracking::mech_eye::CaptureMode mechCaptureMode,
        bool telescopicConcurrentHikC,
        BundleCaptureOptions options);

signals:
    void bundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle);
    void stateChanged(scan_tracking::vision::VisionPipelineState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private slots:
    void onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result);
    void onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void onHikCameraCImageReceived(
        scan_tracking::vision::CaptureType type,
        QString cameraIp,
        QString filePath,
        qint64 fileSize);
    void onHikCameraCCaptureCompleted(
        scan_tracking::vision::CaptureType type,
        QString cameraIp,
        QByteArray imageData);

private:
    struct PendingCaptureContext {
        bool active = false;
        bool mechDone = false;
        bool hikADone = false;
        bool hikBDone = false;
        bool hikCDone = false;
        bool useCxp = false;
        bool useHikCameraC = false;
        bool hikCTriggerOnly = false;
        quint64 mechRequestId = 0;
        quint64 hikARequestId = 0;
        quint64 hikBRequestId = 0;
        QString hikCameraCIp;
        scan_tracking::mech_eye::MechEyeService* activeMechService = nullptr;
        scan_tracking::vision::MultiCameraCaptureBundle bundle;
    };

    static void registerMetaTypes();
    void setState(VisionPipelineState state, const QString& description);
    void startPendingHikCapture();
    void startPendingHikCameraCCapture();
    void triggerHikCameraCConcurrent(bool triggerOnly);
    void completeHikCameraCCapture(const QString& imagePath);
    void onHikCameraCCaptureTimeout();
    void finishBundleIfReady();
    void emitBundleFinished(MultiCameraCaptureBundle bundle);
    void joinLbPoseThread();
    void enqueueLbPoseJob(MultiCameraCaptureBundle bundle);
    void ensureLbPoseWorkerRunning();
    void lbPoseWorkerLoop();

    struct LbPoseJob {
        MultiCameraCaptureBundle bundle;
        scan_tracking::common::LbPoseConfig lbConfig;
    };

    scan_tracking::mech_eye::MechEyeService* m_mechEyeTelescopicService = nullptr;
    scan_tracking::mech_eye::MechEyeService* m_mechEyeArmService = nullptr;
    HikCxpCameraService* m_hikCameraAService = nullptr;
    HikCxpCameraService* m_hikCameraBService = nullptr;
    HikCameraCController* m_hikCameraCController = nullptr;
    scan_tracking::common::VisionConfig m_config;
    scan_tracking::common::LbPoseConfig m_lbPoseConfig;
    PendingCaptureContext m_pending;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    bool m_processing = false;
    VisionPipelineState m_state = VisionPipelineState::Idle;
    // LB 后台线程 + 队列：热路径只投递任务，不在主线程 join。
    std::shared_ptr<std::atomic_bool> m_acceptLbResults;
    std::mutex m_lbPoseThreadMutex;
    std::condition_variable m_lbPoseQueueCv;
    std::deque<LbPoseJob> m_lbPoseQueue;
    std::thread m_lbPoseThread;
    bool m_lbPoseWorkerStopping = false;
    bool m_lbPoseWorkerRunning = false;
};

}  // namespace vision
}  // namespace scan_tracking
