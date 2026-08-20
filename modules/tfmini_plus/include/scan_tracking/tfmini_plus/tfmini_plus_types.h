#pragma once

#include <QtCore/QMetaType>
#include <QtCore/QString>

namespace scan_tracking {
namespace tfmini_plus {

enum class TfminiPlusRuntimeState {
    Idle = 0,
    Opening,
    Ready,
    Failed,
    Stopped,
};

struct TfminiPlusOpenConfig {
    QString portName;
    int baudRate = 115200;
    bool logFrames = false;
    /// 日志/信号标识，如 TF1、TF2；空则用默认 [TfminiPlus]。
    QString deviceLabel;
};

/// 串口 9 字节数据帧解析结果（不含 Byte6/7 温度）。
struct TfminiPlusFrame {
    int distanceCm = 0;
    int strength = 0;
    bool checksumValid = false;
    /// 手册约定：Strength < 100 或 == 65535 时 Dist 不可靠（常为 0）。
    bool isReliable = false;
};

}  // namespace tfmini_plus
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::tfmini_plus::TfminiPlusRuntimeState)
Q_DECLARE_METATYPE(scan_tracking::tfmini_plus::TfminiPlusOpenConfig)
