#pragma once

#include <map>
#include <memory>

#include <QtCore/QStringList>

#include "scan_tracking/flow_control/handlers/itask_handler.h"

namespace scan_tracking {
namespace flow_control {

class TaskHandlerRegistry {
public:
    TaskHandlerRegistry();

    ITaskHandler* handlerForOffset(int offset) const;
    QStringList registeredTriggerNames() const;
    QStringList enabledTriggerNames(const common::StationProfile& profile) const;
    int handlerCount() const;

private:
    void registerHandler(std::unique_ptr<ITaskHandler> handler);

    std::map<int, std::unique_ptr<ITaskHandler>> m_handlers;
};

}  // namespace flow_control
}  // namespace scan_tracking
