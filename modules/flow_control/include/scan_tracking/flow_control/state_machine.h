#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QSet>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

#include <atomic>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/plc_task_host.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"
#include "scan_tracking/flow_control/scan_segment_persist_worker.h"
#include "scan_tracking/flow_control/station3_inspection.h"
#include "scan_tracking/flow_control/task_handler_context.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace mech_eye {
class MechEyeService;
}
namespace vision {
class VisionPipelineService;
class HikCameraCController;
}
namespace flow_control {

class TaskHandlerRegistry;

enum class AppState {
    Init,
    Ready,
    Scanning,
    Error,
};

Q_DECLARE_METATYPE(scan_tracking::flow_control::AppState)

/// HMI 路径级进度事件字段（event.path.*，对齐第一工位显控约定）
struct ScanPathEventInfo {
    int pathId = 0;
    QString pathName;
    int pathIndex = 0;       ///< 在 enabledPathIds 中的序号，1-based
    int pathCount = 0;       ///< 启用路径总数
    QString inspectionType;  ///< 当前路径的 algorithm 标识
    int totalPoints = 0;
    quint16 resultCode = 0;  ///< finished 时：1=成功
};

/// HMI status.system.scanPathProgress 快照
struct ScanPathProgressSnapshot {
    QVector<int> enabledPathIds;
    int currentPathId = 0;
    QString currentPathName;
    QVector<int> completedPathIds;
    int pathCount = 0;
    bool allPathsComplete = false;
};

}  // namespace flow_control
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::flow_control::ScanPathEventInfo)

namespace scan_tracking {
namespace flow_control {

class StateMachine : public QObject, public PlcTaskHost {
    Q_OBJECT

public:
    explicit StateMachine(
        modbus::ModbusService* modbusService,
        mech_eye::MechEyeService* mechEyeTelescopicService = nullptr,
        mech_eye::MechEyeService* mechEyeArmService = nullptr,
        vision::VisionPipelineService* visionPipelineService = nullptr,
        vision::HikCameraCController* hikCameraCController = nullptr,
        QObject* parent = nullptr);
    ~StateMachine();

    void start();
    void stop();

    // 下列访问器刻意非内联：StateMachine 成员布局常随路径进度等字段变更，
    // 调用方（如 hmi_server）若仅增量重编会按旧偏移读到垃圾值并闪退。
    AppState currentState() const;
    protocol::IpcState ipcState() const;
    protocol::Stage currentStage() const;
    quint16 alarmLevel() const;
    quint16 alarmCode() const;
    quint16 warnCode() const;
    quint16 progress() const;

    const QVector<quint16>& lastCommandBlock() const;
    protocol::registers::Pose6f robotTcpPose() const;
    quint16 robotStatusWord() const;

    void setAlarm(quint16 level, quint16 code, const QString& message);
    bool reportPersonZoneAlarm(bool alarm);

    /// 基于当前段缓存同步评估。主线程走 try_lock（繁忙立即返回），避免卡死 GUI/Modbus。
    InspectionResult evaluateCachedInspection(quint32 taskId = 0) const;
    /// 最近一次后台（或同步）检测真结果；无结果时 resultCode=0。
    InspectionResult lastInspectionResult() const;
    bool hasLastInspectionResult() const { return m_hasLastInspectionResult; }

    const ScanSegmentCache& scanSegmentCache() const { return m_scanSegmentCache; }

    /// 供 HMI status.system.scanPathProgress 使用的路径进度快照
    ScanPathProgressSnapshot scanPathProgressSnapshot() const;

    // --- PlcTaskHost（供 Handler 调用）---
    modbus::ModbusService* modbusService() const override;
    mech_eye::MechEyeService* mechEyeService() const override;
    mech_eye::MechEyeService* mechEyeTelescopicService() const override;
    mech_eye::MechEyeService* mechEyeArmService() const override;
    vision::VisionPipelineService* visionPipelineService() const override;
    vision::HikCameraCController* hikCameraCController() const override;
    bool isModbusConnected() const override;

    bool completeActiveTask(
        quint16 resultCode,
        protocol::AckState finalAckState = protocol::AckState::Completed,
        bool dataValid = true) override;

    void publishIpcStatus() override;
    void setTaskProgress(quint16 progress) override;

    PoseSourceResult resolveLoadGraspPoseSource() const override;
    PoseSourceResult resolveUnloadCalcPoseSource() const override;
    void writeLoadGraspResult(const PoseSourceResult& pose) override;
    void writeUnloadCalcResult(const PoseSourceResult& pose) override;
    void writeFloatPlaceholder(int startOffset, float value) override;
    void writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text) override;
    void clearInspectionResultRegisters() override;
    bool writeSelfCheckFailWords(const QVector<quint16>& failWords) override;
    bool clearScanSegmentDoneRegisters() override;
    bool clearIpcSafetyActionWord() override;

    void completeScanSegmentCapture(
        quint16 resultCode,
        int imageCount,
        int cloudFrameCount,
        protocol::AckState finalAckState,
        bool dataValid) override;
    void notifyScanStarted(int segmentIndex, quint32 taskId) override;

    InspectionResult evaluateInspectionForActiveTask() const override;
    void finishInspection(const InspectionResult& result) override;
    void releaseInspectionAndSolveInBackground() override;
    void startCodeReadCapture() override;
    void startSelfCheckCapture() override;

    void resetScanSegmentCache() override;
    /// 切路径时只清段缓存，保留本次运行实例落盘目录。
    void clearScanSegmentCacheForPathSwitch();
    void resetSafetyInterlockState() override;
    void executeResultResetTask() override;
    void maybeAdvancePathOnNewCycleStart(int localIndex) override;

    void notifyLoadGraspFinished(quint16 resultCode, const PoseSourceResult& pose) override;
    void notifyUnloadCalcFinished(quint16 resultCode, const PoseSourceResult& pose) override;
    void notifyPoseCheckFinished(
        bool success,
        quint16 resultCode,
        double poseDeviationMm,
        const QVector<double>& rt,
        const QString& message) override;
    void notifySelfCheckFinished(quint16 resultCode, quint16 failWord0) override;
    void notifyCodeReadFinished(quint16 resultCode, const QString& codeValue) override;
    void notifyResultResetFinished(quint16 resultCode) override;

signals:
    void stateChanged(AppState newState);
    void protocolEvent(const QString& message);
    void scanStarted(int segmentIndex, quint32 taskId);
    void scanFinished(int segmentIndex, quint16 resultCode, int imageCount, int cloudFrameCount);
    void inspectionFinished(quint16 resultCode, quint16 ngReasonWord0, quint16 ngReasonWord1,
                            quint16 measureItemCount,
                            const InspectionMeasurement& measurement,
                            const QString& message);
    void inspectionResultReady(const InspectionResult& result);
    void poseCheckFinished(bool success, quint16 resultCode, double poseDeviationMm,
                           const QVector<double>& rt, const QString& message);
    void loadGraspFinished(quint16 resultCode, float x, float y, float z,
                           float rx, float ry, float rz);
    void unloadCalcFinished(quint16 resultCode, float x, float y, float z,
                            float rx, float ry, float rz);
    void selfCheckFinished(quint16 resultCode, quint16 failWord0);
    void codeReadFinished(quint16 resultCode, const QString& codeValue);
    void resultResetFinished(quint16 resultCode);

    /// 当前扫描路径开始（首段采集或切路后），供 HMI 高亮进行中路径
    void pathStarted(const ScanPathEventInfo& info);
    /// 当前扫描路径完成（检测成功并即将/已经切走）
    void pathFinished(const ScanPathEventInfo& info);
    /// 全部启用路径均已完成
    void scanPathsAllFinished(const QVector<int>& completedPathIds, int pathCount);
    /// 路径进度复位（如 Trig_ResultReset）
    void pathProgressReset(const QString& reason);

private slots:
    void pollPlcState();
    void handleRegistersRead(int startAddress, const QVector<quint16>& values);
    void onRegisterReadFailed(int startAddress, const QString& errorString);
    void onRegisterWriteFailed(int startAddress, const QString& errorString);
    void onModbusConnected();
    void onModbusDisconnected();
    void onModbusError(const QString& errorString);
    void onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message);
    void onVisionPipelineFatalError(vision::VisionErrorCode code, QString message);
    void onBundleCaptureFinished(vision::MultiCameraCaptureBundle bundle);
    void onProcessTimeout();
    void onHikOcrTextReceived(QString cameraIp, QString text);

private:
    struct InspectionSummary {
        quint16 resultCode = 1;
        quint16 ngReasonWord0 = 0;
        quint16 ngReasonWord1 = 0;
        quint16 measureItemCount = 0;
    };

    void setState(AppState newState);
    void processTrigger(const protocol::TriggerDefinition& trigger, const QVector<quint16>& commandBlock);
    void rejectDisabledTrigger(const protocol::TriggerDefinition& trigger);
    void executeActiveTask();

    void sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState);
    void sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode);
    void resetPlcOutputRegisters();
    void publishHeartbeat();
    /// @param force 为 true 时即使当前 Trig 仍为 1 也收尾（用于完成期间 Trig 已拉低后又重新置位）
    void finalizeCompletedTaskIfTriggerReleased(const QVector<quint16>& commandBlock, bool force = false);
    void clearActiveTask();

    const protocol::TriggerDefinition* selectPendingTrigger(
        const QVector<quint16>& commandBlock,
        const QVector<quint16>& previousCommandBlock) const;

    void recordModbusFailure(quint16 alarmCode, const QString& message);
    void resetModbusFailureCounter();
    void enterFaultState(quint16 alarmCode, const QString& message, bool abortCurrentTask, bool notifyPlc);
    void abortActiveTaskForFault(quint16 resultCode);
    bool writeIpcSafetyActionWord();

    void writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount);
    void writeTelescopicScanResult(int segmentIndex, int imageCount, int cloudFrameCount);
    void writeInspectionResult(const InspectionSummary& summary);
    int resolveExpectedScanSegmentCount() const;

    bool isActiveCodeReadTrigger() const;
    void finishCodeRead(quint16 resultCode, const QString& codeValue, const QString& message = QString());
    /// 段扫兼编号已收尾后，迟到的 OCR 只写编号寄存器，不再动 Ack/Res。
    void applyLateCodeReadOcr(const QString& cameraIp, const QString& codeValue);
    /// 编号段扫 Ack 已写出后，若 PLC 迟迟不拉低 Trig，延时强制收尾以免整线卡死。
    void scheduleCodeReadScanFinalizeWatchdog(int trigOffset);
    void finishSelfCheckCapture(const vision::MultiCameraCaptureBundle& bundle);
    struct IncrementalWeldTask {
        int pathId = 0;
        QString runKey;
        common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
        int localIndex = 0;
        std::shared_future<IncrementalWeldSegmentResult> future;
    };

    /// 当前路径检测成功后：清空段缓存/扫描完成寄存器，并自动切到下一条启用路径（不依赖 PLC ResultReset）。
    void prepareNextScanPathAfterSuccess();
    /// 路径齐套且为可后台测量算法时：立刻抽快照并启动后台解算（不切路、不 ACK Inspection）。
    /// @return 是否实际启动了解算
    bool maybeStartInspectionSolveWhenQuotaComplete(const QString& triggerLabel);
    /// weld_section 每段采集成功后立即投递；设备内串行、设备间并行。
    bool scheduleIncrementalWeldSegment(
        common::ScanDeviceKind device,
        int segmentIndex,
        const QString& triggerLabel);
    void resetIncrementalWeldState();
    /// 离开当前路径前兜底：若尚未提前解算且已齐套，则后台跑算法并写 PLC 假成功。
    void maybeAutoRunInspectionBeforeLeavingPath();
    void markCurrentPathInspectionDone();
    bool alreadyInspectedCurrentPathRun() const;
    QString currentInspectionRunKey() const;
    void publishInspectionOutcome(const InspectionResult& result, const QString& triggerLabel);
    /// 真结果仅内存 + Qt/HMI，不写 Modbus。
    void publishInspectionOutcomeToHmiOnly(const InspectionResult& result, const QString& triggerLabel);
    /// 将路径算法结果追加写入 run 目录下 result.txt。
    void saveInspectionResultTxt(
        const InspectionResult& result,
        const QString& runCaptureRootHint = QString());
    /// 向 PLC 写入检测通道假成功（Res/相关字），便于放行流程。
    void writeFakeInspectionPlcSuccess();
    InspectionQuota buildActiveInspectionQuota() const;
    static bool isBackgroundMeasurableAlgorithm(const QString& algorithm);
    /// 快照缓存后后台解算（真结果经 generation 校验后仅推 HMI）。
    /// 单线程串行：最多 1 个执行中 + 2 个 pending（FIFO）；队列满时丢最旧未执行快照。
    void startBackgroundInspectionSolve(
        InspectionCloudSnapshot snapshot,
        quint32 taskId,
        const InspectionQuota& quota,
        quint64 generation,
        const QString& triggerLabel,
        std::vector<IncrementalWeldTask> incrementalWeldTasks = {});
    /// stop/析构前接合后台解算线程，避免 detach 后静态析构竞态。
    void joinBackgroundInspectionSolves();
    void scheduleScanSegmentPersist(
        common::ScanDeviceKind device,
        int segmentIndex,
        const QString& triggerLabel);
    void onScanSegmentPersistFinished(
        common::ScanDeviceKind device,
        int segmentIndex,
        bool ok);
    void applyBackgroundInspectionResult(
        quint64 generation,
        const InspectionResult& result,
        const QString& triggerLabel,
        const QString& runCaptureRoot);

    struct BackgroundInspectionJob {
        InspectionCloudSnapshot cloudSnapshot;
        std::vector<IncrementalWeldTask> incrementalWeldTasks;
        quint32 taskId = 0;
        InspectionQuota quota;
        quint64 generation = 0;
        QString triggerLabel;
        QString runCaptureRoot;
        std::shared_future<void> persistBarrier;
    };
    /// 当前活跃路径的臂+伸缩杆缓存是否已齐套。
    bool isActivePathQuotaComplete() const;
    /// 读取 PLC ScanPathId(40047) 并热切换活跃路径；0=未指定则忽略。
    /// @param onlyOnChange 为 true 时仅在相对 previousCommandBlock 发生变化时切换（避免覆盖 IPC 自动切路）。
    /// @return 是否实际切换成功
    bool applyPlcScanPathId(const QVector<quint16>& commandBlock,
                            const QVector<quint16>& previousCommandBlock = {},
                            bool onlyOnChange = false);

    ScanPathEventInfo buildScanPathEventInfo(int pathId, quint16 resultCode = 0) const;
    void maybeEmitPathStarted(int pathId);
    void maybeEmitPathFinished(int pathId, quint16 resultCode = 1);
    void clearPathProgressTracking(const QString& resetReason);
    /// 丢弃编号软等待、切路锁存等本件运行态（ResultReset / 冷启动）。
    void clearTransientWorkpieceRuntimeState();
    /// 活跃路径回到首条 enabled；成功返回新 pathId，失败返回 0。
    int resetActivePathToFirstEnabled();
    /// 当前活动段扫对应的设备组（臂 / 伸缩杆）。
    common::ScanDeviceKind activeScanDeviceKind() const;
    /// 读取配置的扫描失败清理策略（segment / path / workpiece）。
    QString currentScanFailurePolicy() const;
    /// 对齐工位1：Res>=5 / 段扫超时后按策略清理缓存。
    void applyScanFailurePolicy(
        int pathId,
        common::ScanDeviceKind device,
        int segmentIndex,
        quint16 resultCode);
    /// 检测超时：workpiece 整表清；path 清当前路径段；segment 保留段供重试。
    void applyInspectionTimeoutFailurePolicy();
    /// 递增工件世代，作废在途异步回投（对齐工位1 async generation）。
    quint64 bumpWorkpieceGeneration(const QString& reason);
    quint64 workpieceGeneration() const;
    /// generation 与当前一致则 true；否则打日志并返回 false。
    bool acceptWorkpieceGeneration(quint64 generation, const QString& tag) const;

    quint32 readTaskId(const QVector<quint16>& commandBlock) const;
    quint16 resolveScanSegmentIndex(const QVector<quint16>& commandBlock,
                                    protocol::Stage stage) const;

    modbus::ModbusService* m_modbus = nullptr;
    mech_eye::MechEyeService* m_mechEyeTelescopic = nullptr;
    mech_eye::MechEyeService* m_mechEyeArm = nullptr;
    vision::VisionPipelineService* m_visionPipeline = nullptr;
    vision::HikCameraCController* m_hikCameraCController = nullptr;
    QTimer* m_pollTimer = nullptr;
    QTimer* m_heartbeatTimer = nullptr;
    QTimer* m_timeoutTimer = nullptr;
    std::unique_ptr<TaskHandlerRegistry> m_handlerRegistry;
    AppState m_state = AppState::Init;
    protocol::IpcState m_ipcState = protocol::IpcState::Uninitialized;
    protocol::Stage m_currentStage = protocol::Stage::Idle;
    ActiveTaskState m_activeTask;
    bool m_codeReadPending = false;
    /// 段扫触发编号后已 Ack，仍接受迟到 OCR 写寄存器。
    bool m_codeReadSoftPending = false;
    /// 编号段扫齐套后，等 PLC 拉低 Trig 再切下一路（避免过早清 Done 导致 PLC 认为无应答）。
    bool m_advancePathAfterTriggerRelease = false;
    /// 强制收尾后，该 Trig 必须先回到 0 才允许再次接受（防止 Trig 一直为 1 时重复触发）。
    int m_blockTrigUntilIdleOffset = -1;
    /// 本轮段缓存已做过检测（Trig_Inspection 或切路前自动检测），避免重复跑算法。
    int m_lastInspectedPathId = -1;
    QString m_lastInspectedRunKey;
    /// 后台/同步检测真结果（不依赖 PLC 寄存器）。
    InspectionResult m_lastInspectionResult;
    bool m_hasLastInspectionResult = false;
    QString m_codeReadCameraIp;
    /// 发起编号等待时的工件世代；ResultReset 后迟到 OCR 丢弃。
    quint64 m_codeReadWorkpieceGeneration = 0;
    QSet<int> m_emittedPathStarted;
    QSet<int> m_emittedPathFinished;
    bool m_emittedAllPathsFinished = false;
    /// 工件世代：ResultReset / 整表失败 / stop 递增；异步检测与迟到回调须比对。
    std::atomic<quint64> m_workpieceGeneration{0};
    quint16 m_heartbeatCounter = 0;
    quint16 m_alarmLevel = 0;
    quint16 m_alarmCode = 0;
    quint16 m_warnCode = 0;
    quint16 m_progress = 0;
    quint16 m_ipcSafetyActionWord = 0;
    bool m_personZoneAlarmActive = false;
    bool m_dataValid = false;
    bool m_isPollingPlc = false;
    quint64 m_pollRequestSequence = 0;
    quint64 m_activePollRequestSequence = 0;
    QElapsedTimer m_pollRequestTimer;
    int m_consecutiveModbusFailures = 0;
    QVector<quint16> m_lastCommandBlock;
    protocol::registers::Pose6f m_robotTcpPose;
    ScanSegmentCache m_scanSegmentCache;
    ScanSegmentPersistWorker m_scanPersistWorker;
    /// 最近投递的段落盘任务；齐套快照捕获它，算法线程等待该段完全落盘后再进入 DLL。
    std::shared_future<void> m_latestScanPersistBarrier;
    std::atomic_bool m_stopped{false};
    /// 落盘完成回投门闩：stop 先关闸，已入队的 QueuedConnection 回调不得再碰 this。
    std::shared_ptr<std::atomic_bool> m_persistAcceptResults{
        std::make_shared<std::atomic_bool>(true)};
    /// 后台解算：单线程串行 + pending 深度 2；stop/析构时必须 join。
    static constexpr std::size_t kBgSolvePendingCapacity = 2;
    std::mutex m_bgSolveThreadsMutex;
    std::thread m_bgSolveThread;
    bool m_bgSolveWorkerBusy = false;
    std::deque<BackgroundInspectionJob> m_bgSolvePending;
    /// 后台解算回投门闩；stop 先关闸并 join，保证工作线程访问期间对象仍存活。
    std::atomic_bool m_bgSolveAcceptResults{true};
    /// 当前 weld_section 路径的逐段解算链。shared_future 让齐套收尾只等待、不重算。
    std::mutex m_incrementalWeldMutex;
    std::vector<IncrementalWeldTask> m_incrementalWeldTasks;
    std::shared_future<IncrementalWeldSegmentResult> m_incrementalWeldArmTail;
    std::shared_future<IncrementalWeldSegmentResult> m_incrementalWeldTelescopicTail;
    /// 切路/重置时接管仍在运行的 async shared state，避免其析构阻塞采集线程。
    std::vector<std::shared_future<IncrementalWeldSegmentResult>> m_retiredIncrementalWeldTasks;
};

}  // namespace flow_control
}  // namespace scan_tracking
