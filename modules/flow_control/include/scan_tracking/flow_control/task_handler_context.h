#pragma once

#include <QtCore/QVector>

#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/flow_control/plc_task_host.h"

namespace scan_tracking {
namespace flow_control {

struct ActiveTaskState {
    const protocol::TriggerDefinition* definition = nullptr;
    quint32 taskId = 0;
    quint16 timeoutSeconds = 0;
    bool completionAnnounced = false;
    int scanSegmentIndex = 0;
    int scanSegmentTotal = 0;
    int inspectionPathId = 0;
    quint64 captureRequestId = 0;
    /// 接受本触发时的工件世代；异步回投须比对，ResultReset 后丢弃。
    quint64 workpieceGeneration = 0;
};

struct TaskHandlerContext {
    PlcTaskHost& host;
    const QVector<quint16>& commandBlock;
    ActiveTaskState& activeTask;
};

}  // namespace flow_control
}  // namespace scan_tracking
