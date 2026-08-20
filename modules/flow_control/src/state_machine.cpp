#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/flow_control/task_handler_registry.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

StateMachine::StateMachine(
    modbus::ModbusService* modbusService,
    mech_eye::MechEyeService* mechEyeTelescopicService,
    mech_eye::MechEyeService* mechEyeArmService,
    vision::VisionPipelineService* visionPipelineService,
    vision::HikCameraCController* hikCameraCController,
    QObject* parent)
    : QObject(parent)
    , m_modbus(modbusService)
    , m_mechEyeTelescopic(mechEyeTelescopicService)
    , m_mechEyeArm(mechEyeArmService)
    , m_visionPipeline(visionPipelineService)
    , m_hikCameraCController(hikCameraCController)
    , m_pollTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
    , m_timeoutTimer(new QTimer(this))
    , m_handlerRegistry(std::make_unique<TaskHandlerRegistry>())
    , m_state(AppState::Init)
{
    qRegisterMetaType<scan_tracking::flow_control::ScanPathEventInfo>(
        "scan_tracking::flow_control::ScanPathEventInfo");

    const auto* configMgr = common::ConfigManager::instance();
        const auto flowConfig = configMgr ? configMgr->flowControlConfig()
                                          : common::FlowControlConfig{
                                                100,
                                                1000,
                                                300,
                                                QStringLiteral("segment")};
    if (configMgr) {
        const auto& profile = configMgr->stationProfile();
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Station] StateMachine stationId=")
            << common::stationIdToInt(profile.stationId)
            << QStringLiteral(" workMode=")
            << common::workModeIdToString(profile.defaultWorkMode);
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Station] handlers=")
            << m_handlerRegistry->handlerCount()
            << QStringLiteral(" enabledTriggers=")
            << m_handlerRegistry->enabledTriggerNames(profile).join(QLatin1Char(','));
    }

    m_pollTimer->setInterval(flowConfig.pollIntervalMs);
    m_heartbeatTimer->setInterval(flowConfig.heartbeatIntervalMs);
    m_timeoutTimer->setSingleShot(true);

    connect(m_pollTimer, &QTimer::timeout, this, &StateMachine::pollPlcState);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &StateMachine::publishHeartbeat);
    connect(m_timeoutTimer, &QTimer::timeout, this, &StateMachine::onProcessTimeout);

    if (m_modbus) {
        connect(m_modbus, &modbus::ModbusService::connected, this, &StateMachine::onModbusConnected);
        connect(m_modbus, &modbus::ModbusService::disconnected, this, &StateMachine::onModbusDisconnected);
        connect(m_modbus, &modbus::ModbusService::errorOccurred, this, &StateMachine::onModbusError);
        connect(m_modbus, &modbus::ModbusService::registersRead, this, &StateMachine::handleRegistersRead);
        connect(m_modbus, &modbus::ModbusService::registerReadFailed, this, &StateMachine::onRegisterReadFailed);
        connect(m_modbus, &modbus::ModbusService::registerWriteFailed, this, &StateMachine::onRegisterWriteFailed);
    }

    const auto connectMechEye = [this](mech_eye::MechEyeService* service, const char* label) {
        if (service == nullptr) {
            return;
        }
        connect(
            service,
            &mech_eye::MechEyeService::stateChanged,
            this,
            [label](mech_eye::CameraRuntimeState state, QString desc) {
                qInfo(LOG_FLOW) << label << QStringLiteral("相机状态变更:") << static_cast<int>(state) << desc;
            });
        connect(
            service,
            &mech_eye::MechEyeService::fatalError,
            this,
            &StateMachine::onMechEyeFatalError,
            Qt::QueuedConnection);
    };
    connectMechEye(m_mechEyeTelescopic, "[MechEye-Telescopic]");
    connectMechEye(m_mechEyeArm, "[MechEye-Arm]");

    if (m_visionPipeline) {
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::stateChanged,
            this,
            [](vision::VisionPipelineState state, const QString& description) {
                qInfo(LOG_FLOW) << QStringLiteral("[VisionPipeline] 状态=") << static_cast<int>(state) << description;
            });
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::fatalError,
            this,
            &StateMachine::onVisionPipelineFatalError,
            Qt::QueuedConnection);
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::bundleCaptureFinished,
            this,
            &StateMachine::onBundleCaptureFinished,
            // Direct：同线程免拷贝大包；LB 点云拼接已在 vision LB worker 完成。
            Qt::DirectConnection);
    }

    if (m_hikCameraCController != nullptr) {
        connect(
            m_hikCameraCController,
            &vision::HikCameraCController::ocrTextReceived,
            this,
            &StateMachine::onHikOcrTextReceived,
            Qt::QueuedConnection);
    }

    // 捕获 accept gate 的 shared_ptr：stop 关闸后，已投递的落盘完成回调直接 return，避免 UAF。
    const auto persistAccept = m_persistAcceptResults;
    m_scanPersistWorker.setPersistFinishedHandler(
        [persistAccept, this](common::ScanDeviceKind device, int segmentIndex, bool ok) {
            if (!persistAccept->load(std::memory_order_acquire)) {
                return;
            }
            onScanSegmentPersistFinished(device, segmentIndex, ok);
        });
}

StateMachine::~StateMachine()
{
    stop();
}

void StateMachine::start()
{
    qInfo(LOG_FLOW) << QStringLiteral("状态机启动。");

    // HMI CmdStop → CmdStart/CmdReset 同实例重启：恢复 stop 关掉的门闩，否则后台解算/落盘回投永久跳过。
    m_stopped.store(false, std::memory_order_release);
    m_bgSolveAcceptResults.store(true, std::memory_order_release);
    m_persistAcceptResults->store(true, std::memory_order_release);
    m_scanPersistWorker.restart();
    blockSignals(false);

    clearActiveTask();
    // 无断点续跑：冷启动不得残留上一进程/上一轮段缓存与路径进度。
    bumpWorkpieceGeneration(QStringLiteral("state_machine.start"));
    qInfo(LOG_FLOW).noquote() << QStringLiteral("状态机启动：WorkpieceGen 完成");
    clearTransientWorkpieceRuntimeState();
    qInfo(LOG_FLOW).noquote() << QStringLiteral("状态机启动：临时运行态已清理");
    resetScanSegmentCache();
    resetActivePathToFirstEnabled();
    qInfo(LOG_FLOW).noquote() << QStringLiteral("状态机启动：活跃路径已复位");
    clearPathProgressTracking(QStringLiteral("state_machine.start"));
    m_isPollingPlc = false;
    m_ipcState = protocol::IpcState::Initializing;
    m_currentStage = protocol::Stage::Idle;
    m_alarmLevel = 0;
    m_alarmCode = 0;
    m_warnCode = 0;
    m_progress = 0;
    m_dataValid = false;
    m_consecutiveModbusFailures = 0;
    setState(AppState::Init);
    publishIpcStatus();
    qInfo(LOG_FLOW).noquote() << QStringLiteral("状态机启动完成。");

    // 心跳定时器兼作「相机齐套才首次开 Modbus」门控；已建链后相机抖动不再关监听。
    if (m_heartbeatTimer != nullptr && !m_heartbeatTimer->isActive()) {
        m_heartbeatTimer->start();
    }

    if (m_modbus != nullptr && m_modbus->isConnected()) {
        onModbusConnected();
    }
}

void StateMachine::stop()
{
    const bool firstStop = !m_stopped.exchange(true);
    if (firstStop) {
        bumpWorkpieceGeneration(QStringLiteral("state_machine.stop"));
    }
    // 先关回投门闩再 join：保证后台线程停止访问 StateMachine 后才继续析构。
    m_bgSolveAcceptResults.store(false, std::memory_order_release);
    m_persistAcceptResults->store(false, std::memory_order_release);
    joinBackgroundInspectionSolves();
    m_scanPersistWorker.stopAndJoin();
    if (!firstStop) {
        return;
    }

    blockSignals(true);

    if (m_pollTimer != nullptr) {
        m_pollTimer->stop();
    }
    if (m_heartbeatTimer != nullptr) {
        m_heartbeatTimer->stop();
    }
    if (m_timeoutTimer != nullptr) {
        m_timeoutTimer->stop();
    }

    m_isPollingPlc = false;

    if (m_modbus != nullptr) {
        disconnect(m_modbus, nullptr, this, nullptr);
    }
    if (m_mechEyeTelescopic != nullptr) {
        disconnect(m_mechEyeTelescopic, nullptr, this, nullptr);
    }
    if (m_mechEyeArm != nullptr) {
        disconnect(m_mechEyeArm, nullptr, this, nullptr);
    }
    if (m_visionPipeline != nullptr) {
        disconnect(m_visionPipeline, nullptr, this, nullptr);
    }

    clearActiveTask();

    m_consecutiveModbusFailures = 0;
    m_alarmLevel = 0;
    m_alarmCode = 0;
    m_warnCode = 0;
    m_progress = 0;
    m_dataValid = false;
    m_heartbeatCounter = 0;
    m_ipcState = protocol::IpcState::Uninitialized;
    m_currentStage = protocol::Stage::Idle;
    m_state = AppState::Init;
}

void StateMachine::setState(AppState newState)
{
    if (m_state != newState) {
        m_state = newState;
        if (!m_stopped.load(std::memory_order_acquire)) {
            emit stateChanged(newState);
        }
        qInfo(LOG_FLOW) << QStringLiteral("应用状态切换为：") << static_cast<int>(newState);
    }
}

void StateMachine::executeActiveTask()
{
    if (m_activeTask.definition == nullptr) {
        return;
    }

    ITaskHandler* handler = m_handlerRegistry
        ? m_handlerRegistry->handlerForOffset(m_activeTask.definition->trigOffset)
        : nullptr;
    if (handler == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("拒绝不支持的触发")
            << protocol::triggerName(*m_activeTask.definition);
        setAlarm(2, 624, QStringLiteral("收到不支持的触发"));
        completeActiveTask(9, protocol::AckState::Failed, false);
        return;
    }

    TaskHandlerContext ctx{*this, m_lastCommandBlock, m_activeTask};
    handler->execute(ctx);
}

void StateMachine::clearActiveTask()
{
    m_activeTask = {};
    m_codeReadPending = false;
    m_codeReadCameraIp.clear();
}

modbus::ModbusService* StateMachine::modbusService() const
{
    return m_modbus;
}

bool StateMachine::isModbusConnected() const
{
    return m_modbus != nullptr && m_modbus->isConnected();
}

mech_eye::MechEyeService* StateMachine::mechEyeService() const
{
    return m_mechEyeTelescopic != nullptr ? m_mechEyeTelescopic : m_mechEyeArm;
}

mech_eye::MechEyeService* StateMachine::mechEyeTelescopicService() const
{
    return m_mechEyeTelescopic;
}

mech_eye::MechEyeService* StateMachine::mechEyeArmService() const
{
    return m_mechEyeArm;
}

vision::VisionPipelineService* StateMachine::visionPipelineService() const
{
    return m_visionPipeline;
}

vision::HikCameraCController* StateMachine::hikCameraCController() const
{
    return m_hikCameraCController;
}

AppState StateMachine::currentState() const
{
    return m_state;
}

protocol::IpcState StateMachine::ipcState() const
{
    return m_ipcState;
}

protocol::Stage StateMachine::currentStage() const
{
    return m_currentStage;
}

quint16 StateMachine::alarmLevel() const
{
    return m_alarmLevel;
}

quint16 StateMachine::alarmCode() const
{
    return m_alarmCode;
}

quint16 StateMachine::warnCode() const
{
    return m_warnCode;
}

quint16 StateMachine::progress() const
{
    return m_progress;
}

const QVector<quint16>& StateMachine::lastCommandBlock() const
{
    return m_lastCommandBlock;
}

protocol::registers::Pose6f StateMachine::robotTcpPose() const
{
    return m_robotTcpPose;
}

quint16 StateMachine::robotStatusWord() const
{
    return m_lastCommandBlock.value(protocol::registers::kRobotStatusWord, 0);
}

void StateMachine::setTaskProgress(quint16 progress)
{
    m_progress = progress;
}

void StateMachine::resetSafetyInterlockState()
{
    m_ipcSafetyActionWord = 0;
    m_personZoneAlarmActive = false;
}

void StateMachine::clearTransientWorkpieceRuntimeState()
{
    m_codeReadPending = false;
    m_codeReadSoftPending = false;
    m_codeReadCameraIp.clear();
    m_codeReadWorkpieceGeneration = 0;
    m_advancePathAfterTriggerRelease = false;
    m_lastInspectedPathId = -1;
    m_lastInspectedRunKey.clear();
    m_lastInspectionResult = {};
    m_hasLastInspectionResult = false;
}

quint64 StateMachine::bumpWorkpieceGeneration(const QString& reason)
{
    const quint64 next = m_workpieceGeneration.fetch_add(1, std::memory_order_acq_rel) + 1;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[WorkpieceGen] 已递增 generation=") << next
        << QStringLiteral(" reason=") << reason;
    return next;
}

quint64 StateMachine::workpieceGeneration() const
{
    return m_workpieceGeneration.load(std::memory_order_acquire);
}

bool StateMachine::acceptWorkpieceGeneration(quint64 generation, const QString& tag) const
{
    const quint64 current = workpieceGeneration();
    if (generation == current) {
        return true;
    }
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[WorkpieceGen] 忽略过期回调 tag=") << tag
        << QStringLiteral(" generation=") << generation
        << QStringLiteral(" current=") << current;
    return false;
}

int StateMachine::resetActivePathToFirstEnabled()
{
    auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return 0;
    }

    const QVector<int> ids = cfgMgr->enabledPathIds();
    if (ids.isEmpty()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("ResultReset/启动：无 enabled 路径可复位");
        return 0;
    }

    const int fromPathId = cfgMgr->activePathId();
    const int toPathId = ids.front();
    if (!cfgMgr->setActivePathId(toPathId)) {
        return 0;
    }

    if (fromPathId != toPathId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("活跃路径已复位到首条 enabled：")
            << fromPathId << QStringLiteral(" -> ") << toPathId
            << QStringLiteral("(") << cfgMgr->activePathName() << QStringLiteral(")")
            << QStringLiteral(" algorithm=") << cfgMgr->activePathAlgorithm();
    }
    return toPathId;
}

void StateMachine::executeResultResetTask()
{
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ResultReset：进入新工件复位（清缓存/路径进度/检测标记，路径回首条）");

    const int fromPathId =
        common::ConfigManager::instance() != nullptr
            ? common::ConfigManager::instance()->activePathId()
            : 0;

    resetSafetyInterlockState();
    bumpWorkpieceGeneration(QStringLiteral("result_reset"));
    clearTransientWorkpieceRuntimeState();
    resetScanSegmentCache();
    const int toPathId = resetActivePathToFirstEnabled();
    clearPathProgressTracking(QStringLiteral("result_reset"));

    if (isModbusConnected()) {
        clearScanSegmentDoneRegisters();
        clearInspectionResultRegisters();
        clearIpcSafetyActionWord();
    }

    completeActiveTask(1);
    notifyResultResetFinished(1);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ResultReset 完成：activePathId=")
        << toPathId
        << QStringLiteral(" (was ") << fromPathId << QLatin1Char(')')
        << QStringLiteral("，后续段号 1 按首条路径处理");
}

void StateMachine::notifyLoadGraspFinished(quint16 resultCode, const PoseSourceResult& pose)
{
    emit loadGraspFinished(resultCode, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
}

void StateMachine::notifyUnloadCalcFinished(quint16 resultCode, const PoseSourceResult& pose)
{
    emit unloadCalcFinished(resultCode, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
}

void StateMachine::notifyPoseCheckFinished(
    bool success,
    quint16 resultCode,
    double poseDeviationMm,
    const QVector<double>& rt,
    const QString& message)
{
    emit poseCheckFinished(success, resultCode, poseDeviationMm, rt, message);
}

void StateMachine::notifySelfCheckFinished(quint16 resultCode, quint16 failWord0)
{
    emit selfCheckFinished(resultCode, failWord0);
}

void StateMachine::notifyCodeReadFinished(quint16 resultCode, const QString& codeValue)
{
    emit codeReadFinished(resultCode, codeValue);
}

void StateMachine::notifyResultResetFinished(quint16 resultCode)
{
    // 路径进度清理由 executeResultResetTask / start 显式完成；此处只推 HMI 事件。
    emit resultResetFinished(resultCode);
}

}  // namespace scan_tracking::flow_control
