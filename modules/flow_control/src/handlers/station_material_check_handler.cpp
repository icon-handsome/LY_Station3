#include "scan_tracking/flow_control/handlers/station_material_check_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking::flow_control {

const char* StationMaterialCheckHandler::triggerName() const { return "Trig_StationMaterialCheck"; }
int StationMaterialCheckHandler::trigOffset() const { return 21; }

void StationMaterialCheckHandler::execute(TaskHandlerContext& ctx)
{
    const bool hasModbus = ctx.host.isModbusConnected();
    const bool hasMechEye = ctx.host.mechEyeService() != nullptr;

    if (!hasModbus || !hasMechEye) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("工位检材不可用：")
            << QStringLiteral(" modbus=") << hasModbus
            << QStringLiteral(" mechEye=") << hasMechEye;
        ctx.host.writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("NO"));
        ctx.host.completeActiveTask(5, protocol::AckState::Failed, false);
        return;
    }

    ctx.host.writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("OK"));
    ctx.host.completeActiveTask(1, protocol::AckState::Completed, true);
}

}  // namespace scan_tracking::flow_control
