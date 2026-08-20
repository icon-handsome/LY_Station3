#pragma once

// 视觉域共享类型：Mech-Eye + 海康 CXP 双目组合采集。

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking {
namespace vision {

enum class VisionErrorCode {
    Success = 0,
    NotStarted = 1,
    Busy = 2,
    InvalidConfig = 3,
    CaptureRejected = 4,
    NotImplemented = 5,
    DeviceNotFound = 6,
    DeviceOpenFailed = 7,
    SdkInitFailed = 8,
    UnknownError = 9,
};

enum class VisionPipelineState {
    Idle = 0,
    Ready = 1,
    Capturing = 2,
    Error = 3,
    Stopped = 4,
};

struct PoseMatrix4x4 {
    std::array<float, 16> values = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    bool valid = false;

    bool isValid() const { return valid; }
};

struct HikMonoFrame {
    std::shared_ptr<std::vector<std::uint8_t>> pixels;
    int width = 0;
    int height = 0;
    int stride = 0;
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    QString pixelFormat = QStringLiteral("Mono8");

    bool isValid() const
    {
        return pixels != nullptr && !pixels->empty() && width > 0 && height > 0;
    }
};

struct HikPoseCaptureRequest {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    int timeoutMs = 1000;
};

struct HikPoseCaptureResult {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    VisionErrorCode errorCode = VisionErrorCode::Success;
    QString errorMessage;
    HikMonoFrame frame;
    PoseMatrix4x4 poseMatrix;
    qint64 elapsedMs = 0;

    bool success() const
    {
        return errorCode == VisionErrorCode::Success && frame.isValid();
    }

    bool hasPose() const
    {
        return poseMatrix.isValid();
    }
};

struct HikCameraParams {
    float exposureTimeUs = 0.0f;
    float exposureTimeMinUs = 0.0f;
    float exposureTimeMaxUs = 0.0f;
    bool autoExposureEnabled = false;
    float gainDb = 0.0f;
    float gainMinDb = 0.0f;
    float gainMaxDb = 0.0f;
    bool autoGainEnabled = false;
    float frameRateFps = 0.0f;
    bool frameRateEnabled = false;
    quint32 triggerMode = 0;
    qint64 width = 0;
    qint64 height = 0;
    quint32 pixelFormat = 0;
    QString pixelFormatStr;
    bool valid = false;
    QString errorMessage;
};

enum class CaptureType {
    SurfaceDefect = 0,
    WeldDefect = 1,
    NumberRecognition = 2,
};

/// LB（机械臂扫描段）位姿检测结果，由 runLbPoseDetection 填充
struct LbPoseResult {
    bool invoked = false;       ///< 是否已调用 LB 算法（未调用时 success 无意义）
    bool success = false;       ///< 算法是否成功输出有效位姿
    QString message;            ///< 失败原因或摘要
    int leftImageWidth = 0;
    int leftImageHeight = 0;
    int rightImageWidth = 0;
    int rightImageHeight = 0;
    int framePointCount = 0;    ///< 重建得到的标记点数量
    PoseMatrix4x4 poseMatrix;   ///< 4×4 Rt 位姿矩阵
    QString diagnosticText;     ///< LB 诊断明细
};

struct MultiCameraCaptureRequest {
    quint64 requestId = 0;
    quint32 taskId = 0;
    int segmentIndex = 0;
    bool needMechEye2D = false;
    scan_tracking::mech_eye::CaptureMode mechCaptureMode =
        scan_tracking::mech_eye::CaptureMode::Capture3DOnly;
    QString mechEyeCameraKey;
    int mechEyeTimeoutMs = 5000;
    QString hikCameraAKey;  ///< 右目 CXP-A
    QString hikCameraBKey;  ///< 左目 CXP-B
    int hikTimeoutMs = 1000;
    QString hikCameraCIp;
};

struct MultiCameraCaptureBundle {
    MultiCameraCaptureRequest request;
    scan_tracking::mech_eye::CaptureResult mechEyeResult;
    HikPoseCaptureResult hikCameraAResult;  ///< 右目
    HikPoseCaptureResult hikCameraBResult;  ///< 左目
    QString hikCameraCImagePath;
    bool hikCameraCTriggerOk = false;
    LbPoseResult lbPoseResult;
    /// 落盘后 CXP/海康图像帧可能被清理，但采集状态仍应保持有效。
    bool heavyPayloadsStripped = false;

    bool hikCameraCCaptureOk() const
    {
        return !hikCameraCImagePath.trimmed().isEmpty();
    }

    bool hikCameraCOk() const
    {
        return hikCameraCTriggerOk || hikCameraCCaptureOk();
    }

    bool cxpParticipated() const
    {
        return !request.hikCameraAKey.isEmpty() || !request.hikCameraBKey.isEmpty();
    }

    bool hikCParticipated() const
    {
        return !request.hikCameraCIp.trimmed().isEmpty();
    }

    bool allCamerasOk() const
    {
        const bool mechOk = mechEyeResult.success();
        if (!mechOk) {
            return false;
        }
        if (cxpParticipated() &&
            !(hikCameraAResult.errorCode == VisionErrorCode::Success &&
              hikCameraBResult.errorCode == VisionErrorCode::Success &&
              (heavyPayloadsStripped ||
               (hikCameraAResult.frame.isValid() && hikCameraBResult.frame.isValid())))) {
            return false;
        }
        if (hikCParticipated() &&
            !(heavyPayloadsStripped ? hikCameraCTriggerOk : hikCameraCOk())) {
            return false;
        }
        return true;
    }

    /// 梅卡成功，且已参与的 CXP / 海康 C 均 OK。LB 成败不计入（便于排查与原始云落盘）。
    bool success() const
    {
        return allCamerasOk();
    }

    QString summary() const
    {
        const auto flag = [](bool ok) {
            return ok ? QStringLiteral("成功") : QStringLiteral("失败");
        };
        const auto lbFlag = [](const LbPoseResult& lb) {
            if (!lb.invoked) {
                return QStringLiteral("跳过");
            }
            return lb.success ? QStringLiteral("成功") : QStringLiteral("失败");
        };
        QString hikPart;
        if (cxpParticipated() && hikCParticipated()) {
            hikPart = QStringLiteral("CXP=%1/%2 C=%3")
                          .arg(flag(hikCameraAResult.success()))
                          .arg(flag(hikCameraBResult.success()))
                          .arg(flag(hikCameraCOk()));
        } else if (cxpParticipated()) {
            hikPart = QStringLiteral("CXP=%1/%2")
                          .arg(flag(hikCameraAResult.success()))
                          .arg(flag(hikCameraBResult.success()));
        } else if (hikCParticipated()) {
            hikPart = QStringLiteral("C=%1").arg(flag(hikCameraCOk()));
        } else {
            hikPart = QStringLiteral("无");
        }
        return QStringLiteral(
                   "组合采集 requestId=%1 taskId=%2 段号=%3 梅卡=%4 海康=%5 LB=%6")
            .arg(request.requestId)
            .arg(request.taskId)
            .arg(request.segmentIndex)
            .arg(flag(mechEyeResult.success()))
            .arg(hikPart)
            .arg(lbFlag(lbPoseResult));
    }
};

}  // namespace vision
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::vision::VisionErrorCode)
Q_DECLARE_METATYPE(scan_tracking::vision::VisionPipelineState)
Q_DECLARE_METATYPE(scan_tracking::vision::PoseMatrix4x4)
Q_DECLARE_METATYPE(scan_tracking::vision::HikMonoFrame)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureResult)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureBundle)
Q_DECLARE_METATYPE(scan_tracking::vision::CaptureType)
