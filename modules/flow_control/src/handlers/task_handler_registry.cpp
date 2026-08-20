#include "scan_tracking/flow_control/task_handler_registry.h"

#include "scan_tracking/flow_control/handlers/code_read_handler.h"
#include "scan_tracking/flow_control/handlers/inspection_handler.h"
#include "scan_tracking/flow_control/handlers/load_grasp_handler.h"
#include "scan_tracking/flow_control/handlers/pose_check_handler.h"
#include "scan_tracking/flow_control/handlers/result_reset_handler.h"
#include "scan_tracking/flow_control/handlers/scan_segment_handler.h"
#include "scan_tracking/flow_control/handlers/telescopic_scan_handler.h"
#include "scan_tracking/flow_control/handlers/self_check_handler.h"
#include "scan_tracking/flow_control/handlers/station_material_check_handler.h"
#include "scan_tracking/flow_control/handlers/unload_calc_handler.h"

namespace scan_tracking {
namespace flow_control {

TaskHandlerRegistry::TaskHandlerRegistry()
{
    registerHandler(std::make_unique<LoadGraspHandler>());
    registerHandler(std::make_unique<StationMaterialCheckHandler>());
    registerHandler(std::make_unique<PoseCheckHandler>());
    registerHandler(std::make_unique<TelescopicScanHandler>());
    registerHandler(std::make_unique<ScanSegmentHandler>());
    registerHandler(std::make_unique<InspectionHandler>());
    registerHandler(std::make_unique<UnloadCalcHandler>());
    registerHandler(std::make_unique<SelfCheckHandler>());
    registerHandler(std::make_unique<CodeReadHandler>());
    registerHandler(std::make_unique<ResultResetHandler>());
}

void TaskHandlerRegistry::registerHandler(std::unique_ptr<ITaskHandler> handler)
{
    if (!handler) {
        return;
    }
    m_handlers[handler->trigOffset()] = std::move(handler);
}

ITaskHandler* TaskHandlerRegistry::handlerForOffset(int offset) const
{
    const auto it = m_handlers.find(offset);
    return it == m_handlers.end() ? nullptr : it->second.get();
}

QStringList TaskHandlerRegistry::registeredTriggerNames() const
{
    QStringList names;
    for (const auto& item : m_handlers) {
        names.append(QString::fromLatin1(item.second->triggerName()));
    }
    return names;
}

QStringList TaskHandlerRegistry::enabledTriggerNames(const common::StationProfile& profile) const
{
    QStringList names;
    for (const auto& item : m_handlers) {
        if (item.second->isEnabled(profile)) {
            names.append(QString::fromLatin1(item.second->triggerName()));
        }
    }
    return names;
}

int TaskHandlerRegistry::handlerCount() const
{
    return static_cast<int>(m_handlers.size());
}

}  // namespace flow_control
}  // namespace scan_tracking
