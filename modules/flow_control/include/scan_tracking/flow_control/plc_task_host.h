#pragma once

#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking {
namespace modbus {
class ModbusService;
}
namespace mech_eye {
class MechEyeService;
}
namespace vision {
class VisionPipelineService;
class HikCameraCController;
}
namespace flow_control {

/// 上/下料位姿占位解析结果（LoadGrasp / UnloadCalc Handler 使用）。
struct PoseSourceResult {
    bool available = false;
    bool success = false;
    QString sourceName;
    QString message;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
    float rz = 0.0f;
};

/// Handler 执行 PLC 触发任务时所需的 StateMachine 宿主能力（不含 execute* 入口）。
class PlcTaskHost {
public:
    virtual ~PlcTaskHost() = default;

    virtual modbus::ModbusService* modbusService() const = 0;
    virtual mech_eye::MechEyeService* mechEyeService() const = 0;
    virtual mech_eye::MechEyeService* mechEyeTelescopicService() const = 0;
    virtual mech_eye::MechEyeService* mechEyeArmService() const = 0;
    virtual vision::VisionPipelineService* visionPipelineService() const = 0;
    virtual vision::HikCameraCController* hikCameraCController() const = 0;
    virtual bool isModbusConnected() const = 0;

    virtual bool completeActiveTask(
        quint16 resultCode,
        protocol::AckState finalAckState = protocol::AckState::Completed,
        bool dataValid = true) = 0;

    virtual void publishIpcStatus() = 0;
    virtual void setTaskProgress(quint16 progress) = 0;

    virtual PoseSourceResult resolveLoadGraspPoseSource() const = 0;
    virtual PoseSourceResult resolveUnloadCalcPoseSource() const = 0;
    virtual void writeLoadGraspResult(const PoseSourceResult& pose) = 0;
    virtual void writeUnloadCalcResult(const PoseSourceResult& pose) = 0;
    virtual void writeFloatPlaceholder(int startOffset, float value) = 0;
    virtual void writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text) = 0;
    virtual void clearInspectionResultRegisters() = 0;
    virtual bool writeSelfCheckFailWords(const QVector<quint16>& failWords) = 0;
    virtual bool clearScanSegmentDoneRegisters() = 0;
    virtual bool clearIpcSafetyActionWord() = 0;

    virtual void completeScanSegmentCapture(
        quint16 resultCode,
        int imageCount,
        int cloudFrameCount,
        protocol::AckState finalAckState,
        bool dataValid) = 0;
    virtual void notifyScanStarted(int segmentIndex, quint32 taskId) = 0;

    virtual InspectionResult evaluateInspectionForActiveTask() const = 0;
    /// 同步收尾（编号识别等）：写 PLC + HMI。
    virtual void finishInspection(const InspectionResult& result) = 0;
    /// 测量算法：若齐套时已提前后台解算则仅假成功放行；否则假成功后启动后台解算。真结果只进内存并推 Qt/HMI。
    virtual void releaseInspectionAndSolveInBackground() = 0;

    /// 向机械臂侧海康智能 C 发起编号识别，等待 OCR TCP 回包后写 PLC / 完成任务。
    virtual void startCodeReadCapture() = 0;

    /// 自检：在当前位置发起机械臂 Mech+CXP 组合采集（异步，完成后由 StateMachine 收尾）。
    virtual void startSelfCheckCapture() = 0;

    virtual void resetScanSegmentCache() = 0;
    virtual void resetSafetyInterlockState() = 0;

    /// Trig_ResultReset：新工件复位（清段缓存/路径进度/检测标记，活跃路径回到首条 enabled）。
    /// 与工位1同信号；本工位无断点续跑时也以此为唯一安全开新件入口。
    virtual void executeResultResetTask() = 0;

    /// 当前路径扫描已齐套且 PLC 再次下发本地段号 1 时：清缓存并切到下一路（不依赖 Inspection/ResultReset）。
    virtual void maybeAdvancePathOnNewCycleStart(int localIndex) = 0;

    virtual void notifyLoadGraspFinished(quint16 resultCode, const PoseSourceResult& pose) = 0;
    virtual void notifyUnloadCalcFinished(quint16 resultCode, const PoseSourceResult& pose) = 0;
    virtual void notifyPoseCheckFinished(
        bool success,
        quint16 resultCode,
        double poseDeviationMm,
        const QVector<double>& rt,
        const QString& message) = 0;
    virtual void notifySelfCheckFinished(quint16 resultCode, quint16 failWord0) = 0;
    virtual void notifyCodeReadFinished(quint16 resultCode, const QString& codeValue) = 0;
    virtual void notifyResultResetFinished(quint16 resultCode) = 0;
};

}  // namespace flow_control
}  // namespace scan_tracking
