/**
 * @file hmi_tcp_server.cpp
 * @brief HMI TCP 服务端实现
 */

#include "scan_tracking/hmi_server/hmi_tcp_server.h"
#include "scan_tracking/hmi_server/hmi_session.h"
#include "scan_tracking/hmi_server/hmi_protocol.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/common/config_manager.h"

#include <cmath>

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtCore/QLoggingCategory>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QJsonDocument>
#include <QtCore/QUuid>
#include <QtCore/QPointer>
#include <qdatetime.h>

namespace scan_tracking {
namespace hmi_server {

Q_LOGGING_CATEGORY(LOG_HMI_SERVER, "hmi.server")

// 静态成员初始化：全局日志拦截器回调所需的单例指针和原始日志处理器
HmiTcpServer* HmiTcpServer::s_instance = nullptr;
QtMessageHandler HmiTcpServer::s_previousHandler = nullptr;
bool HmiTcpServer::s_logForwarderInstalled = false;
namespace {

bool g_inTcpSend = false;
bool g_inLogForward = false;

bool isHighFrequencyTcpType(const QString& type)
{
    using namespace msg_type;
    return type == QLatin1String(kStatusSystem)
        || type == QLatin1String(kStatusPlc)
        || type == QLatin1String(kStatusCamera)
        || type == QLatin1String(kStatusDevice)
        || type == QLatin1String(kHeartbeatPing)
        || type == QLatin1String(kHeartbeatPong)
        || type == QLatin1String(kEventLog);
}

bool isStatusPushType(const QString& type)
{
    using namespace msg_type;
    return type == QLatin1String(kStatusSystem)
        || type == QLatin1String(kStatusPlc)
        || type == QLatin1String(kStatusCamera)
        || type == QLatin1String(kStatusDevice);
}

void logStatusPushTx(const QString& type, const QString& msgId, const QJsonObject& payload)
{
    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] → TX") << type << msgId
        << summarizeHmiTracePayload(type, payload);
}

// TODO(hmi): 远程 event.log 转发默认关闭，优先保证 TCP 简洁与主业务打通。
// 若显控需要「远程日志页」，将此处改为 true，并确认 install/uninstall 成对调用。
constexpr bool kForwardQtLogsToHmi = false;

// 相机连/断 event.alarm 专用 code（与 Modbus 900 段区分）
constexpr int kAlarmCodeMechEyeConnect = 910;
constexpr int kAlarmCodeMechEyeDisconnect = 911;
constexpr int kAlarmCodeHikConnect = 912;
constexpr int kAlarmCodeHikDisconnect = 913;
constexpr int kAlarmCodeTelescopicRodFault = 920;
constexpr int kAlarmCodeElectromagnetFault = 921;

bool shouldForwardLogToHmi(QtMsgType type, const QString& category, const QString& msg)
{
    if (g_inTcpSend || g_inLogForward) {
        return false;
    }
    // HMI 模块日志仅本地可见，不包装成 event.log 发到显控
    if (category == QLatin1String("hmi.server") || category == QLatin1String("hmi.session")) {
        return false;
    }
    if (msg.contains(QLatin1String("[TCPIP]"))) {
        return false;
    }
    return type >= QtWarningMsg;
}

}  // namespace

HmiTcpServer::HmiTcpServer(int port, QObject* parent)
    : QObject(parent)
    , m_tcpServer(new QTcpServer(this))
    , m_port(port)
    , m_statusPushTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
{
    connect(m_tcpServer, &QTcpServer::newConnection, this, &HmiTcpServer::onNewConnection);

    m_statusPushTimer->setInterval(500); // 500ms 状态推送
    connect(m_statusPushTimer, &QTimer::timeout, this, &HmiTcpServer::onStatusPushTimer);

    m_heartbeatTimer->setInterval(2000); // 2000ms 心跳
    connect(m_heartbeatTimer, &QTimer::timeout, this, &HmiTcpServer::onHeartbeatTimer);
    
    // 初始化消息处理函数映射表
    initializeMessageHandlers();
}

HmiTcpServer::~HmiTcpServer()
{
    stop();
}

bool HmiTcpServer::start()
{
    if (m_tcpServer->isListening()) {
        return true;
    }

    if (!m_tcpServer->listen(QHostAddress::Any, m_port)) {
        qCritical(LOG_HMI_SERVER) << "HMI TCP 服务端启动失败，端口:" << m_port << "错误:" << m_tcpServer->errorString();
        return false;
    }

    qInfo(LOG_HMI_SERVER) << "HMI TCP 服务端启动成功，监听端口:" << m_port;

    // TODO(hmi): 需要把 IPC 侧 qWarning+ 推到显控 event.log 时，设 kForwardQtLogsToHmi=true
    if (kForwardQtLogsToHmi) {
        installLogForwarder();
    }

    return true;
}

void HmiTcpServer::stop()
{
    if (kForwardQtLogsToHmi) {
        uninstallLogForwarder(); // 卸载日志转发器
    }

    disconnectServiceSignals(); // 断开业务模块信号连接

    m_statusPushTimer->stop(); // 停止状态推送定时器
    m_heartbeatTimer->stop(); // 停止心跳定时器

    if (m_session) {
        m_session->disconnect(); // 断开客户端连接
        m_session->deleteLater();
        m_session = nullptr; // 释放客户端会话指针
    }

    if (m_tcpServer->isListening()) {
        m_tcpServer->close(); // 关闭 TCP 服务器
        qInfo(LOG_HMI_SERVER) << "HMI TCP 服务端已停止";
    }
}

bool HmiTcpServer::isListening() const
{
    return m_tcpServer->isListening(); // 检查 TCP 服务器是否正在监听
}

bool HmiTcpServer::hasClient() const
{
    return m_session != nullptr && m_session->isConnected();
}

void HmiTcpServer::setStateMachine(flow_control::StateMachine* sm) { m_stateMachine = sm; }
void HmiTcpServer::setModbusService(modbus::ModbusService* svc) { m_modbusService = svc; }

void HmiTcpServer::bindServiceSignals()
{
    if (m_serviceSignalsBound) {
        return;
    }

    qRegisterMetaType<scan_tracking::flow_control::AppState>(
        "scan_tracking::flow_control::AppState");
    qRegisterMetaType<scan_tracking::flow_control::ScanPathEventInfo>(
        "scan_tracking::flow_control::ScanPathEventInfo");
    qRegisterMetaType<scan_tracking::mech_eye::CaptureResult>(
        "scan_tracking::mech_eye::CaptureResult");
    qRegisterMetaType<scan_tracking::vision::MultiCameraCaptureBundle>(
        "scan_tracking::vision::MultiCameraCaptureBundle");
    qRegisterMetaType<scan_tracking::flow_control::InspectionResult>(
        "scan_tracking::flow_control::InspectionResult");

    connectStateMachineSignals();
    connectModbusSignals();
    connectMechEyeSignals();
    connectVisionPipelineSignals();
    connectStatusRefreshSignals();
    m_serviceSignalsBound = true;
}

void HmiTcpServer::disconnectServiceSignals()
{
    if (!m_serviceSignalsBound) {
        return;
    }
    if (m_stateMachine) {
        disconnect(m_stateMachine, nullptr, this, nullptr);
    }
    if (m_modbusService) {
        disconnect(m_modbusService, nullptr, this, nullptr);
    }
    if (m_mechEyeTelescopic) {
        disconnect(m_mechEyeTelescopic, nullptr, this, nullptr);
    }
    if (m_mechEyeArm) {
        disconnect(m_mechEyeArm, nullptr, this, nullptr);
    }
    if (m_visionPipeline) {
        disconnect(m_visionPipeline, nullptr, this, nullptr);
    }
    m_serviceSignalsBound = false;
}
void HmiTcpServer::setMechEyeService(mech_eye::MechEyeService* svc)
{
    m_mechEyeTelescopic = svc;
}

void HmiTcpServer::setMechEyeServices(mech_eye::MechEyeService* telescopic, mech_eye::MechEyeService* arm)
{
    m_mechEyeTelescopic = telescopic;
    m_mechEyeArm = arm;
}
void HmiTcpServer::setVisionPipelineService(vision::VisionPipelineService* svc) { m_visionPipeline = svc; }
void HmiTcpServer::setHikCameraServices(vision::HikCxpCameraService* hikA, vision::HikCxpCameraService* hikB,
                                        vision::HikCameraService* hikC) {
    m_hikCameraA = hikA;
    m_hikCameraB = hikB;
    m_hikCameraC = hikC;
}

void HmiTcpServer::setHikCameraCController(vision::HikCameraCController* controller) {
    m_hikCameraCController = controller;
}

void HmiTcpServer::onNewConnection()
{
    while (m_tcpServer->hasPendingConnections()) {
        QTcpSocket* socket = m_tcpServer->nextPendingConnection();
        
        // 单客户端模式：如果已有连接，踢掉旧的或拒绝新的。这里策略是踢掉旧的。
        if (m_session) {
            qWarning(LOG_HMI_SERVER) << "新客户端接入，断开旧客户端会话";
            m_session->disconnect();
            m_session->deleteLater();
            m_session = nullptr;
        }

        m_session = new HmiSession(socket, this);
        qInfo(LOG_HMI_SERVER) << "[TCPIP] 新客户端已接入，IP:" << socket->peerAddress().toString() << "端口:" << socket->peerPort();
        
        connect(m_session, &HmiSession::messageReceived, this, &HmiTcpServer::onMessageReceived);
        connect(m_session, &HmiSession::disconnected, this, &HmiTcpServer::onSessionDisconnected);
        connect(m_session, &HmiSession::heartbeatTimeout, this, &HmiTcpServer::onSessionHeartbeatTimeout);

        invalidateStatusPushCache();
        m_statusPushTimer->start();
        m_heartbeatTimer->start();

        sendToClient(buildEnvelope(QStringLiteral("core.hello"), nextEventId(), QJsonObject()));

        // 延后全量 status / 初始检测帧：避免在 newConnection 同步栈上与采集完成信号重入，
        // 也避免增量编译下立刻触碰大段 StateMachine 字段时放大布局不一致问题。
        QPointer<HmiTcpServer> self(this);
        QTimer::singleShot(0, this, [self]() {
            if (!self || !self->hasClient()) {
                return;
            }
            self->pushAllStatusToClient();
            self->publishInitialInspectionDisplay();
            self->syncCameraConnectivityCache();
            if (self->m_stateMachine) {
                self->syncPlcAuxDeviceAlarmCache(self->m_stateMachine->lastCommandBlock());
            }
        });
    }
}

void HmiTcpServer::onSessionDisconnected()
{
    qInfo(LOG_HMI_SERVER) << "[TCPIP] 客户端连接已断开";
    if (m_session) {
        m_session->deleteLater();
        m_session = nullptr;
    }
    invalidateStatusPushCache();
    m_personZoneAlarmCacheValid = false;
    m_statusPushTimer->stop();
    m_heartbeatTimer->stop();
}

void HmiTcpServer::onSessionHeartbeatTimeout()
{
    qWarning(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 显控心跳超时（约 6s 无有效报文），断开会话");
    if (m_session) {
        m_session->disconnect();
    }
}

// 初始化消息处理函数映射表
void HmiTcpServer::initializeMessageHandlers()
{
    // 连接管理消息
    m_messageHandlers[QString::fromLatin1(msg_type::kHmiHello)]       = &HmiTcpServer::handleHmiHello;  // 欢迎消息
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPing)]  = &HmiTcpServer::handleHeartbeatPing;  // 心跳请求
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPong)]  = &HmiTcpServer::handleHeartbeatPong;  // 心跳响应
    
    // 基础控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStart)]       = &HmiTcpServer::handleCmdStart;  // 启动
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStop)]        = &HmiTcpServer::handleCmdStop;  // 停止
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReset)]       = &HmiTcpServer::handleCmdReset;  // 复位
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdClearAlarm)]  = &HmiTcpServer::handleCmdClearAlarm;  // 清除报警
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetStatus)]   = &HmiTcpServer::handleCmdGetStatus;  // 获取状态
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetConfig)]   = &HmiTcpServer::handleCmdGetConfig;  // 获取配置
    
    // Modbus 控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusConnect)]    = &HmiTcpServer::handleCmdModbusConnect;  // 连接 Modbus
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusDisconnect)] = &HmiTcpServer::handleCmdModbusDisconnect;  // 断开 Modbus
    
    // 相机控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdRefreshCamera)]    = &HmiTcpServer::handleCmdRefreshCamera;  // 刷新相机
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureMechEye)]   = &HmiTcpServer::handleCmdCaptureMechEye;  // 捕获 MechEye
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureBundle)]    = &HmiTcpServer::handleCmdCaptureBundle;  // 捕获 Bundle

    m_messageHandlers[QString::fromLatin1(msg_type::kCmdSetBevelRecipe)] =
        &HmiTcpServer::handleCmdSetBevelRecipe;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdDebugTriggerInspection)] =
        &HmiTcpServer::handleCmdDebugTriggerInspection;
    
    // 直接触发命令（占位实现）
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerScan)]         = &HmiTcpServer::handleCmdTriggerScan;  // 触发扫描
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerInspection)]   = &HmiTcpServer::handleCmdTriggerInspection;  // 触发综合检测
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerSelfCheck)]    = &HmiTcpServer::handleCmdTriggerSelfCheck;  // 触发自检
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerPoseCheck)]    = &HmiTcpServer::handleCmdTriggerPoseCheck;  // 触发位姿校验
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerCodeRead)]     = &HmiTcpServer::handleCmdTriggerCodeRead;  // 触发条码读取
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerResultReset)]  = &HmiTcpServer::handleCmdTriggerResultReset;  // 触发结果复位

    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReportPersonZoneAlarm)] =
        &HmiTcpServer::handleCmdReportPersonZoneAlarm;
    // 兼容显控侧 zone 误拼为 zome
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReportPersonZoneAlarmTypo)] =
        &HmiTcpServer::handleCmdReportPersonZoneAlarm;
}

// 处理接收到的客户端消息，根据 type 字段分发到不同的处理函数
void HmiTcpServer::onMessageReceived(const QJsonObject& message)
{
    // 1. 从 JSON 信封中解析出关键字段
    const QString type = message.value(QLatin1String("type")).toString();
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    if (kHmiTcpVerboseTrace) {
        const QJsonObject payload = message.value(QLatin1String("payload")).toObject();
        qInfo(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] ← RX") << type << msgId
            << summarizeHmiTracePayload(type, payload);
    } else if (!isHighFrequencyTcpType(type)) {
        qDebug(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 接收") << type << msgId;
    }
    
    // ------------------------------------------------------------------
    // 2. 通信协议命令分发路由 (Dispatcher)
    // 使用 QHash 映射表进行 O(1) 时间复杂度的快速查找和分发。
    // 说明：所有 handleXxx 函数内部都会在执行完具体业务后，通过 sendResponse() 
    // 回传与该请求 msgId 完全一致的响应结果给显控界面，形成请求-响应的闭环。
    // ------------------------------------------------------------------
    auto it = m_messageHandlers.find(type);
    if (it != m_messageHandlers.end()) {
        // 找到对应的处理函数，调用成员函数指针
        (this->*it.value())(message);
    } else {
        // 未知消息类型，返回错误响应
        qWarning(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 未知消息类型:") << type << msgId;
        sendResponse(type, msgId, false, QStringLiteral("未知命令类型"));
    }
}

void HmiTcpServer::onStatusPushTimer()
{
    if (!hasClient()) return;
    pushSystemStatus();
    pushPlcStatus();
    pushCameraStatus(); 
    pushDeviceStatus();  // 周期性推送设备在线状态字和故障状态字
}

void HmiTcpServer::onHeartbeatTimer()
{
    if (!hasClient()) return;
    sendToClient(buildEnvelope(QLatin1String(msg_type::kHeartbeatPing), nextEventId(), QJsonObject()));
}

// --- 命令处理实现示例 ---

void HmiTcpServer::handleHmiHello(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kHmiHello), msgId, true, QStringLiteral("欢迎"));
}

void HmiTcpServer::handleHeartbeatPing(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    // 响应 pong
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kHeartbeatPong);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();    
    envelope[QStringLiteral("payload")]   = QJsonObject();
    sendToClient(envelope);
}

void HmiTcpServer::handleHeartbeatPong(const QJsonObject& message)
{
    Q_UNUSED(message)
}

void HmiTcpServer::handleCmdStart(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->start();
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, true, QStringLiteral("状态机已启动"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdStop(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->stop();
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, true, QStringLiteral("状态机已停止"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdGetStatus(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    QJsonObject payload = buildResponsePayload(true, QStringLiteral("状态已返回"));
    payload[QLatin1String("system")] = buildSystemStatusPayload();
    payload[QLatin1String("plc")] = buildPlcStatusPayload();
    payload[QLatin1String("camera")] = buildCameraStatusPayload();
    payload[QLatin1String("device")] = buildDeviceStatusPayload();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kCmdGetStatus), msgId, payload));
    syncStatusPushCacheFromCurrent();
}

void HmiTcpServer::handleCmdReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->start(); // 目前系统使用 start 作为重启/复位的入口
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, true, QStringLiteral("状态机已复位"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdClearAlarm(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->setAlarm(0, 0, QString());
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, true, QStringLiteral("报警已清除"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdGetConfig(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // 从 ConfigManager 单例读取所有配置并序列化为 JSON
    auto* cfgMgr = common::ConfigManager::instance();
    if (!cfgMgr) {
        sendResponse(QLatin1String(msg_type::kCmdGetConfig), msgId, false, QStringLiteral("配置管理器未初始化"));
        return;
    }
    
    QJsonObject configPayload;
    
    // 1. App 配置
    QJsonObject appObj;
    appObj[QLatin1String("version")] = cfgMgr->appConfig().version; // 版本号
    appObj[QLatin1String("environment")] = cfgMgr->appConfig().environment; // 环境
    configPayload[QLatin1String("app")] = appObj; 
    
    // 2. Logger 配置
    QJsonObject loggerObj;
    loggerObj[QLatin1String("level")] = cfgMgr->loggerConfig().level; // 日志级别
    loggerObj[QLatin1String("rotateDays")] = cfgMgr->loggerConfig().rotateDays; // 日志滚动天数
    configPayload[QLatin1String("logger")] = loggerObj;
    
    // 3. Modbus 配置
    QJsonObject modbusObj;
    modbusObj[QLatin1String("host")] = cfgMgr->modbusConfig().host; // 主机
    modbusObj[QLatin1String("port")] = cfgMgr->modbusConfig().port; // 端口
    modbusObj[QLatin1String("unitId")] = cfgMgr->modbusConfig().unitId; // 单元 ID
    modbusObj[QLatin1String("timeoutMs")] = cfgMgr->modbusConfig().timeoutMs; // 超时时间
    modbusObj[QLatin1String("reconnectIntervalMs")] = cfgMgr->modbusConfig().reconnectIntervalMs; // 重连间隔
    configPayload[QLatin1String("modbus")] = modbusObj;
    
    // 4. Camera 配置
    QJsonObject cameraObj;
    cameraObj[QLatin1String("defaultCamera")] = cfgMgr->cameraConfig().defaultCamera; // 默认相机
    cameraObj[QLatin1String("scanTimeoutMs")] = cfgMgr->cameraConfig().scanTimeoutMs; // 扫描超时时间
    configPayload[QLatin1String("camera")] = cameraObj;
    
    // 5. Vision 配置
    QJsonObject visionObj;
    visionObj[QLatin1String("mechEyeCameraKey")] = cfgMgr->visionConfig().mechEyeCameraKey;

    const auto buildDeviceGroupObj =
        [](const scan_tracking::common::VisionDeviceGroupConfig& group) {
            QJsonObject groupObj;
            QJsonObject mechObj;
            mechObj[QLatin1String("logicalName")] = group.mechEye.logicalName;
            mechObj[QLatin1String("cameraKey")] = group.mechEye.cameraKey;
            mechObj[QLatin1String("ipAddress")] = group.mechEye.ipAddress;
            groupObj[QLatin1String("mechEye")] = mechObj;

            QJsonObject hikObj;
            hikObj[QLatin1String("logicalName")] = group.hikCameraC.logicalName;
            hikObj[QLatin1String("cameraKey")] = group.hikCameraC.cameraKey;
            hikObj[QLatin1String("ipAddress")] = group.hikCameraC.ipAddress;
            groupObj[QLatin1String("hikCameraC")] = hikObj;
            groupObj[QLatin1String("hikCameraCFtpDirectory")] = group.hikCameraCFtpDirectory;
            return groupObj;
        };
    visionObj[QLatin1String("telescopicGroup")] =
        buildDeviceGroupObj(cfgMgr->visionConfig().telescopicGroup);
    visionObj[QLatin1String("armGroup")] = buildDeviceGroupObj(cfgMgr->visionConfig().armGroup);
    visionObj[QLatin1String("mechCaptureTimeoutMs")] = cfgMgr->visionConfig().mechCaptureTimeoutMs; // MechEye 捕获超时时间
    visionObj[QLatin1String("hikConnectTimeoutMs")] = cfgMgr->visionConfig().hikConnectTimeoutMs; // Hik 连接超时时间
    visionObj[QLatin1String("hikCaptureTimeoutMs")] = cfgMgr->visionConfig().hikCaptureTimeoutMs; // Hik 捕获超时时间
    visionObj[QLatin1String("hikSdkRoot")] = cfgMgr->visionConfig().hikSdkRoot; // Hik SDK 根目录

    visionObj[QLatin1String("hikCxpEnabled")] = cfgMgr->visionConfig().hikCxpEnabled;
    visionObj[QLatin1String("hikCxpBypassOk")] = cfgMgr->visionConfig().hikCxpBypassOk;
    visionObj[QLatin1String("hikCxpCaptureTimeoutMs")] = cfgMgr->visionConfig().hikCxpCaptureTimeoutMs;

    QJsonObject hikAObj;
    hikAObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCxpCameraA.logicalName;
    hikAObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCxpCameraA.cameraKey;
    hikAObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCxpCameraA.ipAddress;
    hikAObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCxpCameraA.serialNumber;
    hikAObj[QLatin1String("cameraType")] = QStringLiteral("cxp");
    visionObj[QLatin1String("hikCameraA")] = hikAObj;

    QJsonObject hikBObj;
    hikBObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCxpCameraB.logicalName;
    hikBObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCxpCameraB.cameraKey;
    hikBObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCxpCameraB.ipAddress;
    hikBObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCxpCameraB.serialNumber;
    hikBObj[QLatin1String("cameraType")] = QStringLiteral("cxp");
    visionObj[QLatin1String("hikCameraB")] = hikBObj;

    QJsonObject hikCObj;
    hikCObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCameraC.logicalName;
    hikCObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCameraC.cameraKey;
    hikCObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCameraC.ipAddress;
    hikCObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCameraC.serialNumber;
    hikCObj[QLatin1String("accessMode")] = cfgMgr->visionConfig().hikCameraC.accessMode;
    visionObj[QLatin1String("hikCameraC")] = hikCObj;
    
    configPayload[QLatin1String("vision")] = visionObj;
    
    // 6. FlowControl 配置
    QJsonObject flowControlObj;
    flowControlObj[QLatin1String("pollIntervalMs")] = cfgMgr->flowControlConfig().pollIntervalMs; // 轮询间隔
    flowControlObj[QLatin1String("heartbeatIntervalMs")] = cfgMgr->flowControlConfig().heartbeatIntervalMs; // 心跳间隔
    flowControlObj[QLatin1String("simulatedProcessingMs")] = cfgMgr->flowControlConfig().simulatedProcessingMs; // 模拟处理间隔
    flowControlObj[QLatin1String("algorithmEnabled")] = cfgMgr->flowControlConfig().algorithmEnabled;
    configPayload[QLatin1String("flowControl")] = flowControlObj;
    
    // 7. 扫描路径配置（含路径目录 + 当前活跃路径）
    QJsonObject scanPathsObj;
    {
        const int scanPointTotal = cfgMgr->enabledScanPointCount();
        scanPathsObj[QLatin1String("scanSegmentTotal")] = scanPointTotal > 0
            ? scanPointTotal
            : cfgMgr->trackingConfig().scanSegmentTotal;
        scanPathsObj[QLatin1String("activePathId")] = cfgMgr->activePathId();
        scanPathsObj[QLatin1String("activePathName")] = cfgMgr->activePathName();
        scanPathsObj[QLatin1String("activePathAlgorithm")] = cfgMgr->activePathAlgorithm();
        scanPathsObj[QLatin1String("armPointCount")] = cfgMgr->enabledArmPointCount();
        scanPathsObj[QLatin1String("telescopicPointCount")] = cfgMgr->enabledTelescopicPointCount();

        QJsonArray pathsArray;
        for (const auto& path : cfgMgr->scanPathsConfig().scanPaths) {
            if (!path.enabled) {
                continue;
            }
            const QString algorithm = common::ConfigManager::resolvePathAlgorithm(path);
            if (algorithm == QLatin1String("self_check")) {
                continue;
            }
            QJsonObject pathObj;
            pathObj[QLatin1String("pathId")] = path.pathId;
            pathObj[QLatin1String("pathName")] =
                path.name.isEmpty() ? QStringLiteral("路径%1").arg(path.pathId) : path.name;
            pathObj[QLatin1String("inspectionType")] = algorithm;
            pathObj[QLatin1String("algorithm")] = algorithm;
            pathObj[QLatin1String("totalPoints")] = path.totalPoints > 0
                ? path.totalPoints
                : (path.armPointCount + path.telescopicPointCount);
            pathObj[QLatin1String("enabled")] = path.enabled;
            pathsArray.append(pathObj);
        }
        scanPathsObj[QLatin1String("paths")] = pathsArray;
    }
    configPayload[QLatin1String("scanPaths")] = scanPathsObj;

    QJsonObject hmiObj;
    hmiObj[QLatin1String("enabled")] = cfgMgr->hmiConfig().enabled;
    hmiObj[QLatin1String("tcpPort")] = cfgMgr->hmiConfig().tcpPort;
    configPayload[QLatin1String("hmi")] = hmiObj;

    QJsonObject payload = buildResponsePayload(true, QStringLiteral("配置已获取")); // 构建响应Payload
    payload[QLatin1String("config")] = configPayload;
    
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdGetConfig);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    sendToClient(envelope);
}

void HmiTcpServer::handleCmdModbusConnect(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_modbusService) {
        m_modbusService->connectDevice();
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, true, QStringLiteral("正在连接 Modbus"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, false, QStringLiteral("Modbus 服务不可用"));
    }
}

void HmiTcpServer::handleCmdModbusDisconnect(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_modbusService) {
        m_modbusService->disconnectDevice();
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, true, QStringLiteral("已断开 Modbus 连接"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, false, QStringLiteral("Modbus 服务不可用"));
    }
}

void HmiTcpServer::handleCmdRefreshCamera(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // 刷新 MechEye 3D 相机连接状态
    if (m_mechEyeTelescopic) {
        m_mechEyeTelescopic->requestRefreshStatus();
    }
    if (m_mechEyeArm) {
        m_mechEyeArm->requestRefreshStatus();
    }
    if (m_mechEyeTelescopic || m_mechEyeArm) {
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, true, QStringLiteral("相机刷新请求已发送"));
    } else {
        qWarning(LOG_HMI_SERVER) << "[TCPIP] 相机刷新失败：MechEye 服务不可用";
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, false, QStringLiteral("相机服务不可用"));
    }
}

void HmiTcpServer::handleCmdTriggerScan(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerScan), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC 或状态机"));
}

void HmiTcpServer::handleCmdTriggerInspection(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerInspection), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerSelfCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    // TODO(hmi): 显控自检命令已接入接收，后续对接 StateMachine::executeSelfCheckTask 或独立流程，并推送 event.self_check.finished
    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 显控触发自检") << msgId;
    sendResponse(
        QLatin1String(msg_type::kCmdTriggerSelfCheck),
        msgId,
        true,
        QStringLiteral("自检命令已接收，后续执行逻辑待实现"));
}

void HmiTcpServer::handleCmdTriggerPoseCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerPoseCheck), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerCodeRead(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerCodeRead), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerResultReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerResultReset), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdReportPersonZoneAlarm(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QJsonObject payload = message.value(QLatin1String("payload")).toObject();

    const QString requestType = message.value(QLatin1String("type")).toString();

    if (!payload.contains(QLatin1String("alarm"))) {
        qWarning(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 人员区域上报缺少 alarm 字段 type=") << requestType
            << QStringLiteral(" msgId=") << msgId;
        sendResponse(
            requestType.isEmpty() ? QLatin1String(msg_type::kCmdReportPersonZoneAlarm) : requestType,
            msgId,
            false,
            QStringLiteral("缺少必填字段：alarm"));
        return;
    }

    const bool alarm = payload.value(QLatin1String("alarm")).toBool();
    const bool stateChanged =
        !m_personZoneAlarmCacheValid || alarm != m_personZoneAlarm;

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 人员区域上报")
        << QStringLiteral(" alarm=") << (alarm ? QStringLiteral("true(有人)") : QStringLiteral("false(无人)"))
        << QStringLiteral(" msgId=") << msgId
        << (stateChanged ? QStringLiteral(" [状态变化]") : QStringLiteral(" [重复上报]"));

    m_personZoneAlarm = alarm;
    m_personZoneAlarmCacheValid = true;

    bool plcWritten = true;
    if (stateChanged) {
        if (m_stateMachine) {
            plcWritten = m_stateMachine->reportPersonZoneAlarm(alarm);
        } else {
            qWarning(LOG_HMI_SERVER).noquote()
                << QStringLiteral("[TCPIP] 人员区域上报：状态机不可用，无法写 PLC");
            sendResponse(
                requestType.isEmpty() ? QLatin1String(msg_type::kCmdReportPersonZoneAlarm) : requestType,
                msgId,
                false,
                QStringLiteral("状态机不可用"));
            return;
        }
    }

    QString responseMessage;
    if (stateChanged) {
        responseMessage = alarm
            ? QStringLiteral("人员区域报警已上报 PLC")
            : QStringLiteral("人员区域报警已解除");
        if (!plcWritten) {
            responseMessage += QStringLiteral("（PLC 未连接或写入失败）");
        }
    } else {
        responseMessage = QStringLiteral("人员区域状态已接收（与上次相同）");
    }

    sendResponse(
        requestType.isEmpty() ? QLatin1String(msg_type::kCmdReportPersonZoneAlarm) : requestType,
        msgId,
        true,
        responseMessage);
}

void HmiTcpServer::handleCmdCaptureMechEye(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QString cameraKey = message.value(QLatin1String("payload")).toObject().value(QLatin1String("cameraKey")).toString();
    mech_eye::MechEyeService* service = resolveMechEyeService(cameraKey);
    if (service != nullptr) {
        quint64 reqId = service->requestCapture(cameraKey, mech_eye::CaptureMode::Capture3DOnly);
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("采集请求已发送"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureMechEye);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureMechEye), msgId, false, QStringLiteral("相机服务不可用"));
    }
}

void HmiTcpServer::handleCmdCaptureBundle(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_visionPipeline) {
        const QJsonObject payloadObj = message.value(QLatin1String("payload")).toObject();
        const int segmentIndex = payloadObj.value(QLatin1String("segmentIndex")).toInt(1);
        const quint32 taskId = static_cast<quint32>(payloadObj.value(QLatin1String("taskId")).toInt(0));
        bool needMechEye2D = payloadObj.value(QLatin1String("needMechEye2D")).toBool(false);
        if (!payloadObj.contains(QLatin1String("needMechEye2D"))) {
            const auto* configMgr = scan_tracking::common::ConfigManager::instance();
            if (configMgr != nullptr) {
                const auto* point = configMgr->findScanPointByIndex(segmentIndex);
                if (point != nullptr) {
                    needMechEye2D = point->needRotation;
                }
            }
        }
        const auto mechCaptureMode = needMechEye2D
            ? scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D
            : scan_tracking::mech_eye::CaptureMode::Capture3DOnly;
        const QString deviceGroup = payloadObj.value(QLatin1String("deviceGroup")).toString().trimmed();
        const bool telescopicInternal =
            deviceGroup.compare(QStringLiteral("telescopic"), Qt::CaseInsensitive) == 0
            || deviceGroup.compare(QStringLiteral("internal"), Qt::CaseInsensitive) == 0;
        quint64 reqId = m_visionPipeline->requestCaptureBundle(
            segmentIndex, taskId, mechCaptureMode, telescopicInternal);
        
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("组合采集请求已发送"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureBundle);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureBundle), msgId, false, QStringLiteral("视觉流水线不可用"));
    }
}

void HmiTcpServer::handleCmdSetBevelRecipe(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(
        QLatin1String(msg_type::kCmdSetBevelRecipe),
        msgId,
        false,
        QStringLiteral("不支持：cmd.set_bevel_recipe 已废弃（第三工位未配置坡口配方）"));
}

void HmiTcpServer::handleCmdDebugTriggerInspection(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();

    if (m_stateMachine == nullptr) {
        sendResponse(
            QLatin1String(msg_type::kCmdDebugTriggerInspection),
            msgId,
            false,
            QStringLiteral("状态机不可用"));
        return;
    }

    const flow_control::InspectionResult result = m_stateMachine->evaluateCachedInspection();
    publishInspectionResult(result);

    const bool success = result.resultCode == 1;
    sendResponse(
        QLatin1String(msg_type::kCmdDebugTriggerInspection),
        msgId,
        success,
        result.message);
}

// --- 状态推送实现 ---

void HmiTcpServer::invalidateStatusPushCache()
{
    m_systemStatusCache = {};
    m_plcStatusCache = {};
    m_cameraStatusCache = {};
    m_deviceStatusCache = {};
    m_cameraConnectivityCache = {};
    m_plcAuxDeviceAlarmCache = {};
}

bool HmiTcpServer::pushStatusIfChanged(const QString& type, const QJsonObject& payload,
                                       HmiStatusPushCache& slot, bool forcePush)
{
    if (!forcePush && slot.isValid && payload == slot.payload) {
        return false;
    }
    slot.payload = payload;
    slot.isValid = true;
    const QString msgId = nextEventId();
    sendToClient(buildEnvelope(type, msgId, payload));
    if (hasClient() && isStatusPushType(type)) {
        logStatusPushTx(type, msgId, payload);
    }
    return true;
}

void HmiTcpServer::pushAllStatusToClient()
{
    if (!hasClient()) {
        return;
    }
    pushStatusIfChanged(QLatin1String(msg_type::kStatusSystem), buildSystemStatusPayload(),
                        m_systemStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusPlc), buildPlcStatusPayload(),
                        m_plcStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusCamera), buildCameraStatusPayload(),
                        m_cameraStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusDevice), buildDeviceStatusPayload(),
                        m_deviceStatusCache, true);
}

void HmiTcpServer::syncStatusPushCacheFromCurrent()
{
    if (m_stateMachine) {
        m_systemStatusCache.payload = buildSystemStatusPayload();
        m_systemStatusCache.isValid = true;
    }
    if (m_modbusService) {
        m_plcStatusCache.payload = buildPlcStatusPayload();
        m_plcStatusCache.isValid = true;
    }
    m_cameraStatusCache.payload = buildCameraStatusPayload();
    m_cameraStatusCache.isValid = true;
    m_deviceStatusCache.payload = buildDeviceStatusPayload();
    m_deviceStatusCache.isValid = true;
    syncCameraConnectivityCache();
    if (m_stateMachine) {
        syncPlcAuxDeviceAlarmCache(m_stateMachine->lastCommandBlock());
    }
}

void HmiTcpServer::pushSystemStatus()
{
    if (!m_stateMachine) return;
    const QJsonObject payload = buildSystemStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusSystem), payload, m_systemStatusCache);
}

QJsonObject HmiTcpServer::buildSystemStatusPayload() const
{
    QJsonObject payload;
    if (!m_stateMachine) return payload;

    QString appStateStr;
    switch (m_stateMachine->currentState()) {
    case flow_control::AppState::Init:     appStateStr = QStringLiteral("Init"); break;
    case flow_control::AppState::Ready:    appStateStr = QStringLiteral("Ready"); break;
    case flow_control::AppState::Scanning: appStateStr = QStringLiteral("Scanning"); break;
    case flow_control::AppState::Error:    appStateStr = QStringLiteral("Error"); break;
    default:                               appStateStr = QStringLiteral("Unknown"); break;
    }

    // 一律转 int 写入 JSON：避免 quint16 在部分 Qt5 重载下类型歧义；也便于日志 toInt 读。
    int alarmLevel = static_cast<int>(m_stateMachine->alarmLevel());
    int progress = static_cast<int>(m_stateMachine->progress());
    if (alarmLevel < 0 || alarmLevel > 3 || progress < 0 || progress > 100) {
        qWarning(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] status.system 字段异常 alarmLevel=") << alarmLevel
            << QStringLiteral(" progress=") << progress
            << QStringLiteral("（疑似未完整重编导致 StateMachine 布局不一致，请 clean 后全量编译并整包替换）");
        // 防御钳制，避免把垃圾值推给显控；根因仍须全量重编。
        alarmLevel = qBound(alarmLevel, 0, 3);
        progress = qBound(progress, 0, 100);
    }
    payload[QLatin1String("ipcState")] = static_cast<int>(m_stateMachine->ipcState());
    payload[QLatin1String("appState")] = appStateStr;
    payload[QLatin1String("stage")] = static_cast<int>(m_stateMachine->currentStage());
    payload[QLatin1String("alarmLevel")] = alarmLevel;
    payload[QLatin1String("alarmCode")] = static_cast<int>(m_stateMachine->alarmCode());
    payload[QLatin1String("warnCode")] = static_cast<int>(m_stateMachine->warnCode());
    payload[QLatin1String("ipcReady")] = (m_stateMachine->currentState() == flow_control::AppState::Ready) ? 1 : 0;
    payload[QLatin1String("progress")] = progress;
    if (const auto* cfgMgr = scan_tracking::common::ConfigManager::instance()) {
        const auto& profile = cfgMgr->stationProfile();
        payload[QLatin1String("stationId")] = scan_tracking::common::stationIdToInt(profile.stationId);
        payload[QLatin1String("stationName")] = profile.stationName;
        payload[QLatin1String("workMode")] =
            scan_tracking::common::workModeIdToString(profile.defaultWorkMode);
        QJsonArray enabledTriggers;
        for (const auto& trigger : scan_tracking::flow_control::protocol::triggerDefinitions()) {
            if (scan_tracking::flow_control::isTriggerEnabledForProfile(profile, trigger.trigOffset)) {
                enabledTriggers.append(QString::fromLatin1(trigger.name));
            }
        }
        payload[QLatin1String("enabledTriggers")] = enabledTriggers;
    }
    payload[QLatin1String("scanPathProgress")] = buildScanPathProgressPayload();
    return payload;
}

QJsonObject HmiTcpServer::buildScanPathProgressPayload() const
{
    QJsonObject obj;
    if (!m_stateMachine) {
        return obj;
    }

    const flow_control::ScanPathProgressSnapshot snapshot = m_stateMachine->scanPathProgressSnapshot();

    QJsonArray enabledPathIds;
    for (int pathId : snapshot.enabledPathIds) {
        enabledPathIds.append(pathId);
    }
    QJsonArray completedPathIds;
    for (int pathId : snapshot.completedPathIds) {
        completedPathIds.append(pathId);
    }

    obj[QLatin1String("enabledPathIds")] = enabledPathIds;
    obj[QLatin1String("currentPathId")] = snapshot.currentPathId;
    obj[QLatin1String("currentPathName")] = snapshot.currentPathName;
    obj[QLatin1String("completedPathIds")] = completedPathIds;
    obj[QLatin1String("pathCount")] = snapshot.pathCount;
    obj[QLatin1String("allPathsComplete")] = snapshot.allPathsComplete;
    return obj;
}

void HmiTcpServer::pushPlcStatus()
{
    const QJsonObject payload = buildPlcStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusPlc), payload, m_plcStatusCache);
}

QJsonObject HmiTcpServer::buildPlcStatusPayload() const
{
    QJsonObject payload;
    payload[QLatin1String("modbusConnected")] = m_modbusService && m_modbusService->isConnected();
    if (const auto* cfgMgr = scan_tracking::common::ConfigManager::instance()) {
        const auto& profile = cfgMgr->stationProfile();
        payload[QLatin1String("stationId")] = scan_tracking::common::stationIdToInt(profile.stationId);
        payload[QLatin1String("stationName")] = profile.stationName;
        payload[QLatin1String("stationWorkMode")] =
            scan_tracking::common::workModeIdToString(profile.defaultWorkMode);
    }
    if (!m_modbusService) {
        return payload;
    }

    if (m_stateMachine) {
        namespace regs = flow_control::protocol::registers;
        const auto& cb = m_stateMachine->lastCommandBlock();
        if (cb.size() > regs::kArmScanSegmentIndex) {
            payload[QLatin1String("plcHeartbeat")]    = static_cast<int>(cb.value(regs::kPlcHeartbeat));
            payload[QLatin1String("plcSystemState")]  = static_cast<int>(cb.value(regs::kPlcSystemState));
            payload[QLatin1String("workMode")]        = static_cast<int>(cb.value(regs::kStationWorkMode));
            payload[QLatin1String("flowEnable")]      = static_cast<int>(cb.value(regs::kFlowEnable));
            payload[QLatin1String("safetyWord")]      = static_cast<int>(cb.value(regs::kSafetyStatusWord));
            payload[QLatin1String("taskId")]          = static_cast<int>(
                (static_cast<quint32>(cb.value(regs::kTaskIdHigh)) << 16) |
                static_cast<quint32>(cb.value(regs::kTaskIdLow)));
            payload[QLatin1String("productType")]     = static_cast<int>(cb.value(regs::kProductType));
            payload[QLatin1String("recipeId")]        = static_cast<int>(cb.value(regs::kRecipeId));
            const int armSegmentIndex = static_cast<int>(
                regs::plcAnalogToUInt16(cb.value(regs::kArmScanSegmentIndex), 0));
            const int telescopicSegmentIndex = (cb.size() > regs::kTelescopicScanSegmentIndex)
                ? static_cast<int>(
                      regs::plcAnalogToUInt16(cb.value(regs::kTelescopicScanSegmentIndex), 0))
                : 0;
            payload[QLatin1String("armScanSegmentIndex")] = armSegmentIndex;
            payload[QLatin1String("telescopicScanSegmentIndex")] = telescopicSegmentIndex;
            // 兼容旧字段：默认回传机械臂段号（AO47）
            payload[QLatin1String("scanSegmentIndex")] = armSegmentIndex;
            // scanSegmentTotal / 路径配额：来自 scan_paths 活跃路径（activePathId）
            const auto* cfgMgr = scan_tracking::common::ConfigManager::instance();
            if (cfgMgr != nullptr) {
                const int scanPointTotal = cfgMgr->enabledScanPointCount();
                payload[QLatin1String("scanSegmentTotal")] = scanPointTotal > 0
                    ? scanPointTotal
                    : cfgMgr->trackingConfig().scanSegmentTotal;
                payload[QLatin1String("activePathId")] = cfgMgr->activePathId();
                payload[QLatin1String("activePathName")] = cfgMgr->activePathName();
                payload[QLatin1String("activePathAlgorithm")] = cfgMgr->activePathAlgorithm();
                payload[QLatin1String("armPointCount")] = cfgMgr->enabledArmPointCount();
                payload[QLatin1String("telescopicPointCount")] = cfgMgr->enabledTelescopicPointCount();
            } else {
                payload[QLatin1String("scanSegmentTotal")] = 0;
                payload[QLatin1String("activePathId")] = 0;
                payload[QLatin1String("activePathName")] = QString();
                payload[QLatin1String("activePathAlgorithm")] = QString();
                payload[QLatin1String("armPointCount")] = 0;
                payload[QLatin1String("telescopicPointCount")] = 0;
            }
            if (cb.size() > regs::kRobotStatusWord) {
                payload[QLatin1String("robotStatusWord")] =
                    static_cast<int>(cb.value(regs::kRobotStatusWord));
            }
            if (cb.size() > regs::kEstopButtonStatus) {
                payload[QLatin1String("telescopicRodStatus")] =
                    static_cast<int>(cb.value(regs::kTelescopicRodStatus));
                payload[QLatin1String("rollerSetFreqHz")] =
                    static_cast<int>(cb.value(regs::kRollerSetFreqHz));
                payload[QLatin1String("rollerRunFreqHz")] =
                    static_cast<int>(cb.value(regs::kRollerRunFreqHz));
                payload[QLatin1String("electromagnetStatus")] =
                    static_cast<int>(cb.value(regs::kElectromagnetStatus));
                payload[QLatin1String("estopButtonStatus")] =
                    static_cast<int>(cb.value(regs::kEstopButtonStatus));
            }
        }
    }
    return payload;
}

void HmiTcpServer::syncPlcAuxDeviceAlarmCache(const QVector<quint16>& commandBlock)
{
    namespace regs = flow_control::protocol::registers;
    if (commandBlock.size() <= regs::kEstopButtonStatus) {
        m_plcAuxDeviceAlarmCache = {};
        return;
    }
    m_plcAuxDeviceAlarmCache.telescopicRodStatus = commandBlock.value(regs::kTelescopicRodStatus);
    m_plcAuxDeviceAlarmCache.electromagnetStatus = commandBlock.value(regs::kElectromagnetStatus);
    m_plcAuxDeviceAlarmCache.valid = true;
}

void HmiTcpServer::emitPlcAuxDeviceAlarm(const QString& message, int code, int level)
{
    if (!hasClient()) {
        return;
    }
    QJsonObject payload;
    payload[QLatin1String("message")] = message;
    payload[QLatin1String("level")] = level;
    payload[QLatin1String("code")] = code;
    payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
}

void HmiTcpServer::checkPlcAuxDeviceAlarms(const QVector<quint16>& commandBlock)
{
    namespace regs = flow_control::protocol::registers;
    if (commandBlock.size() <= regs::kEstopButtonStatus) {
        return;
    }

    const int rodStatus = commandBlock.value(regs::kTelescopicRodStatus);
    const int magnetStatus = commandBlock.value(regs::kElectromagnetStatus);

    const auto emitIfFault = [this](int current, int previous, bool cacheValid, int code, const QString& label) {
        if (current != 2) {
            return;
        }
        if (!cacheValid || previous != 2) {
            emitPlcAuxDeviceAlarm(QStringLiteral("PLC辅机报警：%1").arg(label), code, 2);
        }
    };

    emitIfFault(rodStatus, m_plcAuxDeviceAlarmCache.telescopicRodStatus,
                m_plcAuxDeviceAlarmCache.valid, kAlarmCodeTelescopicRodFault,
                QStringLiteral("伸缩杆故障"));
    emitIfFault(magnetStatus, m_plcAuxDeviceAlarmCache.electromagnetStatus,
                m_plcAuxDeviceAlarmCache.valid, kAlarmCodeElectromagnetFault,
                QStringLiteral("电磁吸盘报警"));

    m_plcAuxDeviceAlarmCache.telescopicRodStatus = rodStatus;
    m_plcAuxDeviceAlarmCache.electromagnetStatus = magnetStatus;
    m_plcAuxDeviceAlarmCache.valid = true;
}

void HmiTcpServer::pushCameraStatus()
{
    const QJsonObject payload = buildCameraStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusCamera), payload, m_cameraStatusCache);
    checkCameraConnectivityEdges();
}

bool HmiTcpServer::mechEyeServiceConnected(const mech_eye::MechEyeService* service) const
{
    if (service == nullptr) {
        return false;
    }
    const auto state = service->state();
    return state != mech_eye::CameraRuntimeState::Idle
        && state != mech_eye::CameraRuntimeState::Error;
}

mech_eye::MechEyeService* HmiTcpServer::resolveMechEyeService(const QString& cameraKey) const
{
    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    const QString key = cameraKey.trimmed();
    if (configMgr != nullptr && !key.isEmpty()) {
        const auto& visionConfig = configMgr->visionConfig();
        if (key == visionConfig.telescopicGroup.mechEye.cameraKey.trimmed()) {
            return m_mechEyeTelescopic;
        }
        if (key == visionConfig.armGroup.mechEye.cameraKey.trimmed()) {
            return m_mechEyeArm;
        }
    }
    return m_mechEyeTelescopic != nullptr ? m_mechEyeTelescopic : m_mechEyeArm;
}

void HmiTcpServer::syncCameraConnectivityCache()
{
    if (m_mechEyeTelescopic) {
        m_cameraConnectivityCache.mechEyeTelescopic = mechEyeServiceConnected(m_mechEyeTelescopic);
    }
    if (m_mechEyeArm) {
        m_cameraConnectivityCache.mechEyeArm = mechEyeServiceConnected(m_mechEyeArm);
    }
    if (m_hikCameraA) {
        m_cameraConnectivityCache.hikA = m_hikCameraA->isConnected();
    }
    if (m_hikCameraB) {
        m_cameraConnectivityCache.hikB = m_hikCameraB->isConnected();
    }
    if (m_hikCameraC || m_hikCameraCController) {
        const auto* configMgr = scan_tracking::common::ConfigManager::instance();
        if (configMgr != nullptr && m_hikCameraCController != nullptr) {
            const auto& visionConfig = configMgr->visionConfig();
            m_cameraConnectivityCache.hikCTelescopic = hikCameraCConnected(
                visionConfig.telescopicGroup.hikCameraC.ipAddress);
            m_cameraConnectivityCache.hikCArm = hikCameraCConnected(
                visionConfig.armGroup.hikCameraC.ipAddress);
        } else {
            m_cameraConnectivityCache.hikCTelescopic = hikCameraCConnected();
        }
    }
    m_cameraConnectivityCache.valid = true;
}

void HmiTcpServer::emitCameraConnectivityAlarm(const QString& deviceLabel, bool connected, int code)
{
    if (!hasClient()) {
        return;
    }
    QJsonObject payload;
    payload[QLatin1String("message")] = connected
        ? QStringLiteral("%1已连接").arg(deviceLabel)
        : QStringLiteral("%1已断开").arg(deviceLabel);
    payload[QLatin1String("level")] = connected ? 1 : 2;
    payload[QLatin1String("code")] = code;
    payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
}

void HmiTcpServer::checkCameraConnectivityEdges()
{
    if (!hasClient()) {
        return;
    }
    if (!m_cameraConnectivityCache.valid) {
        syncCameraConnectivityCache();
        return;
    }

    const auto checkMechEdge = [this](mech_eye::MechEyeService* service,
                                      const QString& label,
                                      bool& cachedConnected) {
        if (service == nullptr) {
            return;
        }
        const bool nowConnected = mechEyeServiceConnected(service);
        if (nowConnected != cachedConnected) {
            emitCameraConnectivityAlarm(
                label,
                nowConnected,
                nowConnected ? kAlarmCodeMechEyeConnect : kAlarmCodeMechEyeDisconnect);
            cachedConnected = nowConnected;
        }
    };
    checkMechEdge(
        m_mechEyeTelescopic,
        QStringLiteral("梅卡相机[伸缩杆]"),
        m_cameraConnectivityCache.mechEyeTelescopic);
    checkMechEdge(
        m_mechEyeArm,
        QStringLiteral("梅卡相机[机械臂]"),
        m_cameraConnectivityCache.mechEyeArm);

    if (m_hikCameraA) {
        const bool nowConnected = m_hikCameraA->isConnected();
        if (nowConnected != m_cameraConnectivityCache.hikA) {
            const QString label = QStringLiteral("海康相机 [%1]").arg(m_hikCameraA->roleName());
            emitCameraConnectivityAlarm(
                label,
                nowConnected,
                nowConnected ? kAlarmCodeHikConnect : kAlarmCodeHikDisconnect);
            m_cameraConnectivityCache.hikA = nowConnected;
        }
    }

    if (m_hikCameraB) {
        const bool nowConnected = m_hikCameraB->isConnected();
        if (nowConnected != m_cameraConnectivityCache.hikB) {
            const QString label = QStringLiteral("海康相机 [%1]").arg(m_hikCameraB->roleName());
            emitCameraConnectivityAlarm(
                label,
                nowConnected,
                nowConnected ? kAlarmCodeHikConnect : kAlarmCodeHikDisconnect);
            m_cameraConnectivityCache.hikB = nowConnected;
        }
    }

    const auto checkHikCEdge = [this](const QString& ip, const QString& label, bool& cachedConnected) {
        if (ip.trimmed().isEmpty()) {
            return;
        }
        const bool nowConnected = hikCameraCConnected(ip);
        if (nowConnected != cachedConnected) {
            emitCameraConnectivityAlarm(
                label,
                nowConnected,
                nowConnected ? kAlarmCodeHikConnect : kAlarmCodeHikDisconnect);
            cachedConnected = nowConnected;
        }
    };
    if (m_hikCameraC || m_hikCameraCController) {
        const auto* configMgr = scan_tracking::common::ConfigManager::instance();
        if (configMgr != nullptr) {
            const auto& visionConfig = configMgr->visionConfig();
            checkHikCEdge(
                visionConfig.telescopicGroup.hikCameraC.ipAddress,
                QStringLiteral("海康智能相机[伸缩杆]"),
                m_cameraConnectivityCache.hikCTelescopic);
            checkHikCEdge(
                visionConfig.armGroup.hikCameraC.ipAddress,
                QStringLiteral("海康智能相机[机械臂]"),
                m_cameraConnectivityCache.hikCArm);
        } else {
            checkHikCEdge(
                QString(),
                QStringLiteral("海康智能相机"),
                m_cameraConnectivityCache.hikCTelescopic);
        }
    }
}

QJsonObject HmiTcpServer::buildCameraStatusPayload() const
{
    QJsonObject payload;

    const auto buildMechObj = [this](const mech_eye::MechEyeService* service) {
        QJsonObject mechEyeObj;
        if (service == nullptr) {
            mechEyeObj[QLatin1String("connected")] = false;
            return mechEyeObj;
        }
        mechEyeObj[QLatin1String("roleName")] = service->roleName();
        mechEyeObj[QLatin1String("state")] = static_cast<int>(service->state());
        mechEyeObj[QLatin1String("connected")] = mechEyeServiceConnected(service);
        return mechEyeObj;
    };

    payload[QLatin1String("mechEyeTelescopic")] = buildMechObj(m_mechEyeTelescopic);
    payload[QLatin1String("mechEyeArm")] = buildMechObj(m_mechEyeArm);
    if (m_mechEyeTelescopic) {
        payload[QLatin1String("mechEye")] = buildMechObj(m_mechEyeTelescopic);
    }
    
    if (m_hikCameraA) {
        QJsonObject hikAObj;
        hikAObj[QLatin1String("roleName")] = m_hikCameraA->roleName();
        hikAObj[QLatin1String("connected")] = m_hikCameraA->isConnected();
        payload[QLatin1String("hikA")] = hikAObj;
    }
    
    if (m_hikCameraB) {
        QJsonObject hikBObj;
        hikBObj[QLatin1String("roleName")] = m_hikCameraB->roleName();
        hikBObj[QLatin1String("connected")] = m_hikCameraB->isConnected();
        payload[QLatin1String("hikB")] = hikBObj;
    }

    if (m_hikCameraC) {
        QJsonObject hikCObj;
        hikCObj[QLatin1String("roleName")] = m_hikCameraC->roleName();
        hikCObj[QLatin1String("connected")] = hikCameraCConnected();
        payload[QLatin1String("hikC")] = hikCObj;
    }

    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr != nullptr && m_hikCameraCController) {
        const auto& visionConfig = configMgr->visionConfig();
        QJsonObject hikCTelescopicObj;
        hikCTelescopicObj[QLatin1String("ipAddress")] =
            visionConfig.telescopicGroup.hikCameraC.ipAddress;
        hikCTelescopicObj[QLatin1String("connected")] = hikCameraCConnected(
            visionConfig.telescopicGroup.hikCameraC.ipAddress);
        payload[QLatin1String("hikCTelescopic")] = hikCTelescopicObj;

        QJsonObject hikCArmObj;
        hikCArmObj[QLatin1String("ipAddress")] = visionConfig.armGroup.hikCameraC.ipAddress;
        hikCArmObj[QLatin1String("connected")] =
            hikCameraCConnected(visionConfig.armGroup.hikCameraC.ipAddress);
        payload[QLatin1String("hikCArm")] = hikCArmObj;
    }
    
    if (m_visionPipeline) {
        QJsonObject pipelineObj;
        pipelineObj[QLatin1String("state")] = static_cast<int>(m_visionPipeline->state());
        payload[QLatin1String("pipeline")] = pipelineObj;
    }
    return payload;
}

bool HmiTcpServer::hikCameraCConnected(const QString& cameraIp) const
{
    const QString normalizedIp = cameraIp.trimmed();
    if (m_hikCameraCController && m_hikCameraCController->isStarted()) {
        if (!normalizedIp.isEmpty()) {
            return m_hikCameraCController->isCameraConnected(normalizedIp);
        }
        if (m_hikCameraCController->isCameraConnectedToTcp()) {
            return true;
        }
    }
    return m_hikCameraC && m_hikCameraC->isConnected();
}

bool HmiTcpServer::hikCameraCConnected() const
{
    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr != nullptr && m_hikCameraCController && m_hikCameraCController->isStarted()) {
        const auto& visionConfig = configMgr->visionConfig();
        return hikCameraCConnected(visionConfig.telescopicGroup.hikCameraC.ipAddress)
            || hikCameraCConnected(visionConfig.armGroup.hikCameraC.ipAddress);
    }
    return hikCameraCConnected(QString());
}

void HmiTcpServer::pushDeviceStatus()
{
    const QJsonObject payload = buildDeviceStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusDevice), payload, m_deviceStatusCache);
}

QJsonObject HmiTcpServer::buildDeviceStatusPayload() const
{
    // onlineWord0 / faultWord0 位定义见显控 TCP 协议 §2.4（bit5 原 Tracking，现表示扫描流水线已启动）
    constexpr int kBitIpcCore = 0;
    constexpr int kBitHmiClient = 1;
    constexpr int kBitMechEye = 2;
    constexpr int kBitVisionPipeline = 3;
    constexpr int kBitHik2d = 4;
    constexpr int kBitScanPipeline = 5;
    constexpr int kBitModbus = 6;
    constexpr int kBitAlarmWarn = 7;

    quint16 onlineWord0 = 0;
    quint16 faultWord0 = 0;

    onlineWord0 |= (1u << kBitIpcCore);

    if (hasClient()) {
        onlineWord0 |= (1u << kBitHmiClient);
    }

    const bool mechTelescopicOnline = mechEyeServiceConnected(m_mechEyeTelescopic);
    const bool mechArmOnline = mechEyeServiceConnected(m_mechEyeArm);
    if (mechTelescopicOnline || mechArmOnline) {
        onlineWord0 |= (1u << kBitMechEye);
    }
    if ((m_mechEyeTelescopic
         && m_mechEyeTelescopic->state() == mech_eye::CameraRuntimeState::Error)
        || (m_mechEyeArm && m_mechEyeArm->state() == mech_eye::CameraRuntimeState::Error)) {
        faultWord0 |= (1u << kBitMechEye);
    }

    if (m_visionPipeline && m_visionPipeline->state() == vision::VisionPipelineState::Ready) {
        onlineWord0 |= (1u << kBitVisionPipeline);
    }
    if (m_visionPipeline && m_visionPipeline->state() == vision::VisionPipelineState::Error) {
        faultWord0 |= (1u << kBitVisionPipeline);
    }

    const bool hikAOnline = m_hikCameraA && m_hikCameraA->isConnected();
    const bool hikBOnline = m_hikCameraB && m_hikCameraB->isConnected();
    const bool hikCOnline = hikCameraCConnected();
    const bool hasAnyHikService = m_hikCameraA || m_hikCameraB || m_hikCameraC;
    if (hikAOnline || hikBOnline || hikCOnline) {
        onlineWord0 |= (1u << kBitHik2d);
    }
    if (hasAnyHikService && !hikAOnline && !hikBOnline && !hikCOnline) {
        faultWord0 |= (1u << kBitHik2d);
    }

    if (m_visionPipeline && m_visionPipeline->isStarted()) {
        onlineWord0 |= (1u << kBitScanPipeline);
    }

    const bool modbusOnline = m_modbusService && m_modbusService->isConnected();
    if (modbusOnline) {
        onlineWord0 |= (1u << kBitModbus);
    }
    if (m_modbusService && !modbusOnline) {
        faultWord0 |= (1u << kBitModbus);
    }

    if (m_stateMachine) {
        const auto ipcState = m_stateMachine->ipcState();
        const auto appState = m_stateMachine->currentState();
        const quint16 alarmLevel = m_stateMachine->alarmLevel();

        if (ipcState == flow_control::protocol::IpcState::Fault
            || appState == flow_control::AppState::Error) {
            faultWord0 |= (1u << kBitIpcCore);
        }
        if (alarmLevel >= 3) {
            faultWord0 |= (1u << kBitIpcCore);
        }
        if (alarmLevel >= 2) {
            faultWord0 |= (1u << kBitAlarmWarn);
        }
    }

    QJsonObject payload;
    payload[QLatin1String("onlineWord0")] = static_cast<int>(onlineWord0);
    payload[QLatin1String("faultWord0")] = static_cast<int>(faultWord0);
    return payload;
}

// --- 事件连接 ---

void HmiTcpServer::connectStateMachineSignals()
{
    if (!m_stateMachine) {
        return;
    }

    qRegisterMetaType<QVector<double>>("QVector<double>");

    // 绑定核心业务报警事件：将状态机发出的协议级错误拦截并封装为 event.alarm 向远端推送
    const bool okProtocol = connect(m_stateMachine, &flow_control::StateMachine::protocolEvent, this,
        [this](const QString& message) {
        QJsonObject payload;
        payload[QLatin1String("message")] = message;
        payload[QLatin1String("level")] = m_stateMachine->alarmLevel();
        payload[QLatin1String("code")] = m_stateMachine->alarmCode();
        payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    }, Qt::UniqueConnection);
    if (!okProtocol) {
        qCritical(LOG_HMI_SERVER) << "connect protocolEvent 失败，请完整重编 flow_control 与 hmi_server";
    }

    // 绑定扫描分段启动事件：告知显控界面哪一段扫描正在拍摄
    connect(m_stateMachine, &flow_control::StateMachine::scanStarted, this,
        [this](int segmentIndex, quint32 taskId) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("taskId")] = static_cast<int>(taskId);
        if (const auto* cfgMgr = scan_tracking::common::ConfigManager::instance()) {
            payload[QLatin1String("pathId")] = cfgMgr->activePathId();
            payload[QLatin1String("pathName")] = cfgMgr->activePathName();
            const QString purpose = cfgMgr->pointPurpose(segmentIndex);
            if (!purpose.isEmpty()) {
                payload[QLatin1String("purpose")] = purpose;
            }
        }
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanStarted), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 绑定扫描分段完成事件：推送当前段是否采图成功及有效帧数
    connect(m_stateMachine, &flow_control::StateMachine::scanFinished, this,
        [this](int segmentIndex, quint16 resultCode, int imageCount, int cloudFrameCount) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("resultCode")] = resultCode; // 1 表示成功
        payload[QLatin1String("imageCount")] = imageCount;
        payload[QLatin1String("cloudFrameCount")] = cloudFrameCount;
        if (const auto* cfgMgr = scan_tracking::common::ConfigManager::instance()) {
            payload[QLatin1String("pathId")] = cfgMgr->activePathId();
            payload[QLatin1String("pathName")] = cfgMgr->activePathName();
        }
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    connect(m_stateMachine, &flow_control::StateMachine::pathStarted, this,
        [this](const flow_control::ScanPathEventInfo& info) {
        QJsonObject payload;
        payload[QLatin1String("pathId")] = info.pathId;
        payload[QLatin1String("pathName")] = info.pathName;
        payload[QLatin1String("pathIndex")] = info.pathIndex;
        payload[QLatin1String("pathCount")] = info.pathCount;
        payload[QLatin1String("inspectionType")] = info.inspectionType;
        payload[QLatin1String("algorithm")] = info.inspectionType;
        payload[QLatin1String("totalPoints")] = info.totalPoints;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventPathStarted), nextEventId(), payload));
        pushSystemStatus();
    }, Qt::UniqueConnection);

    connect(m_stateMachine, &flow_control::StateMachine::pathFinished, this,
        [this](const flow_control::ScanPathEventInfo& info) {
        QJsonObject payload;
        payload[QLatin1String("pathId")] = info.pathId;
        payload[QLatin1String("pathName")] = info.pathName;
        payload[QLatin1String("pathIndex")] = info.pathIndex;
        payload[QLatin1String("pathCount")] = info.pathCount;
        payload[QLatin1String("inspectionType")] = info.inspectionType;
        payload[QLatin1String("algorithm")] = info.inspectionType;
        payload[QLatin1String("totalPoints")] = info.totalPoints;
        if (info.resultCode > 0) {
            payload[QLatin1String("resultCode")] = static_cast<int>(info.resultCode);
        }
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventPathFinished), nextEventId(), payload));
        pushSystemStatus();
    }, Qt::UniqueConnection);

    connect(m_stateMachine, &flow_control::StateMachine::scanPathsAllFinished, this,
        [this](const QVector<int>& completedPathIds, int pathCount) {
        QJsonObject payload;
        QJsonArray completedArray;
        for (int pathId : completedPathIds) {
            completedArray.append(pathId);
        }
        payload[QLatin1String("completedPathIds")] = completedArray;
        payload[QLatin1String("pathCount")] = pathCount;
        sendToClient(buildEnvelope(
            QLatin1String(msg_type::kEventScanPathsAllFinished), nextEventId(), payload));
        pushSystemStatus();
    }, Qt::UniqueConnection);

    connect(m_stateMachine, &flow_control::StateMachine::pathProgressReset, this,
        [this](const QString& reason) {
        QJsonObject payload;
        payload[QLatin1String("reason")] = reason;
        sendToClient(buildEnvelope(
            QLatin1String(msg_type::kEventPathProgressReset), nextEventId(), payload));
        pushSystemStatus();
    }, Qt::UniqueConnection);

    connect(
        m_stateMachine,
        &flow_control::StateMachine::inspectionResultReady,
        this,
        &HmiTcpServer::publishInspectionResult,
        Qt::UniqueConnection);

    // 位姿校验完成
    connect(m_stateMachine, &flow_control::StateMachine::poseCheckFinished, this,
        [this](bool success, quint16 resultCode, double poseDeviationMm, const QVector<double>& rt, const QString& message) {
        QJsonObject payload;
        QJsonArray rtArray;
        for (double value : rt) {
            rtArray.append(value);
        }
        payload[QLatin1String("success")] = success;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("poseDeviationMm")] = poseDeviationMm;
        payload[QLatin1String("rt")] = rtArray;
        payload[QLatin1String("message")] = message;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventPoseCheckFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 上料抓取完成
    connect(m_stateMachine, &flow_control::StateMachine::loadGraspFinished, this,
        [this](quint16 resultCode, float x, float y, float z, float rx, float ry, float rz) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("x")] = static_cast<double>(x);
        payload[QLatin1String("y")] = static_cast<double>(y);
        payload[QLatin1String("z")] = static_cast<double>(z);
        payload[QLatin1String("rx")] = static_cast<double>(rx);
        payload[QLatin1String("ry")] = static_cast<double>(ry);
        payload[QLatin1String("rz")] = static_cast<double>(rz);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventLoadGraspFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 卸料计算完成
    connect(m_stateMachine, &flow_control::StateMachine::unloadCalcFinished, this,
        [this](quint16 resultCode, float x, float y, float z, float rx, float ry, float rz) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("x")] = static_cast<double>(x);
        payload[QLatin1String("y")] = static_cast<double>(y);
        payload[QLatin1String("z")] = static_cast<double>(z);
        payload[QLatin1String("rx")] = static_cast<double>(rx);
        payload[QLatin1String("ry")] = static_cast<double>(ry);
        payload[QLatin1String("rz")] = static_cast<double>(rz);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventUnloadCalcFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 自检完成
    connect(m_stateMachine, &flow_control::StateMachine::selfCheckFinished, this,
        [this](quint16 resultCode, quint16 failWord0) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("failWord0")] = failWord0;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventSelfCheckFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 条码读取完成
    connect(m_stateMachine, &flow_control::StateMachine::codeReadFinished, this,
        [this](quint16 resultCode, const QString& codeValue) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("codeValue")] = codeValue;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventCodeReadFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 结果复位完成
    connect(m_stateMachine, &flow_control::StateMachine::resultResetFinished, this,
        [this](quint16 resultCode) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventResultResetFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);
}

void HmiTcpServer::connectModbusSignals()
{
    if (!m_modbusService) return;

    connect(m_modbusService, &modbus::ModbusService::connected, this, [this]() {
        QJsonObject payload;
        payload[QLatin1String("message")] = QStringLiteral("Modbus connected");
        payload[QLatin1String("level")] = 0;
        payload[QLatin1String("code")] = 0;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });

    connect(m_modbusService, &modbus::ModbusService::disconnected, this, [this]() {
        QJsonObject payload;
        payload[QLatin1String("message")] = QStringLiteral("Modbus disconnected");
        payload[QLatin1String("level")] = 3;
        payload[QLatin1String("code")] = 900;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });

    connect(m_modbusService, &modbus::ModbusService::errorOccurred, this, [this](const QString& errorString) {
        QJsonObject payload;
        payload[QLatin1String("message")] = errorString;
        payload[QLatin1String("level")] = 2;
        payload[QLatin1String("code")] = 901;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });
}

void HmiTcpServer::connectMechEyeSignals()
{
    const auto connectOne = [this](mech_eye::MechEyeService* service) {
        if (service == nullptr) {
            return;
        }
        // 同线程 Auto/Direct：只取元数据，禁止 QueuedConnection 排队整份 CaptureResult（含点云/纹理）。
        connect(service, &mech_eye::MechEyeService::captureFinished, this,
            [this](const scan_tracking::mech_eye::CaptureResult& result) {
                if (!hasClient()) {
                    return;
                }
                QJsonObject payload;
                payload[QLatin1String("requestId")] = static_cast<qint64>(result.requestId);
                payload[QLatin1String("cameraKey")] = result.cameraKey;
                payload[QLatin1String("pointCount")] = result.pointCloud.pointCount;
                payload[QLatin1String("width")] = result.pointCloud.width;
                payload[QLatin1String("height")] = result.pointCloud.height;
                payload[QLatin1String("elapsedMs")] = static_cast<int>(result.elapsedMs);
                payload[QLatin1String("errorCode")] = static_cast<int>(result.errorCode);
                sendToClient(buildEnvelope(QLatin1String(msg_type::kEventImageCaptured), nextEventId(), payload));
            });

        connect(service, &mech_eye::MechEyeService::fatalError, this,
            [this](scan_tracking::mech_eye::CaptureErrorCode code, const QString& message) {
                if (!hasClient()) {
                    return;
                }
                QJsonObject payload;
                payload[QLatin1String("message")] = message;
                payload[QLatin1String("level")] = 3;
                payload[QLatin1String("code")] = static_cast<int>(code);
                sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
                pushDeviceStatus();
            });
    };
    connectOne(m_mechEyeTelescopic);
    connectOne(m_mechEyeArm);
}

void HmiTcpServer::connectStatusRefreshSignals()
{
    if (m_stateMachine) {
        connect(m_stateMachine, &flow_control::StateMachine::stateChanged, this, [this]() {
            if (hasClient()) {
                pushSystemStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    if (m_modbusService) {
        connect(m_modbusService, &modbus::ModbusService::connected, this, [this]() {
            if (hasClient()) {
                pushPlcStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
        connect(m_modbusService, &modbus::ModbusService::disconnected, this, [this]() {
            if (hasClient()) {
                pushPlcStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
        connect(m_modbusService, &modbus::ModbusService::registersRead, this,
                [this](int, const QVector<quint16>& values) {
            checkPlcAuxDeviceAlarms(values);
        }, Qt::UniqueConnection);
    }

    const auto connectMechState = [this](mech_eye::MechEyeService* service) {
        if (service == nullptr) {
            return;
        }
        connect(service, &mech_eye::MechEyeService::stateChanged, this,
                [this](mech_eye::CameraRuntimeState, QString) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    };
    connectMechState(m_mechEyeTelescopic);
    connectMechState(m_mechEyeArm);

    if (m_visionPipeline) {
        connect(m_visionPipeline, &vision::VisionPipelineService::stateChanged, this,
                [this](vision::VisionPipelineState, const QString&) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    auto refreshCameraOnHik = [this](QString, QString, QString) {
        if (hasClient()) {
            pushCameraStatus();
            pushDeviceStatus();
        }
    };
    if (m_hikCameraA) {
        connect(m_hikCameraA, &vision::HikCxpCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }
    if (m_hikCameraB) {
        connect(m_hikCameraB, &vision::HikCxpCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }
    if (m_hikCameraC) {
        connect(m_hikCameraC, &vision::HikCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }

    if (m_hikCameraCController) {
        connect(m_hikCameraCController, &vision::HikCameraCController::stateChanged, this,
                [this](vision::HikCameraCState, const QString&) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }
}

void HmiTcpServer::connectVisionPipelineSignals()
{
    if (!m_visionPipeline) return;

    // 同线程只取摘要字段，避免 QueuedConnection 拷贝整包 MultiCameraCaptureBundle。
    connect(m_visionPipeline, &vision::VisionPipelineService::bundleCaptureFinished, this,
        [this](const scan_tracking::vision::MultiCameraCaptureBundle& bundle) {
        if (!hasClient()) {
            return;
        }
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = bundle.request.segmentIndex;
        payload[QLatin1String("taskId")] = static_cast<int>(bundle.request.taskId);
        payload[QLatin1String("mechOk")] = bundle.mechEyeResult.success();
        payload[QLatin1String("hikAOk")] = bundle.hikCameraAResult.success();
        payload[QLatin1String("hikBOk")] = bundle.hikCameraBResult.success();
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventBundleCaptured), nextEventId(), payload));
    });
}

// --- 日志转发实现（默认关闭，见 kForwardQtLogsToHmi）---

void HmiTcpServer::installLogForwarder()
{
    if (s_logForwarderInstalled) {
        return;
    }

    // 保存当前实例指针，供全局回调函数使用
    s_instance = this;
    
    // 拦截全局 Qt 日志输出，并保存原有处理器以支持链式调用
    s_previousHandler = qInstallMessageHandler([](QtMsgType type, const QMessageLogContext& context, const QString& msg) {
        const QString category = QString::fromLatin1(context.category ? context.category : "default");

        if (s_instance && s_instance->hasClient() && shouldForwardLogToHmi(type, category, msg)) {
            QJsonObject payload;
            int severity = 2;
            switch (type) {
            case QtWarningMsg: severity = 2; break;
            case QtCriticalMsg: severity = 3; break;
            case QtFatalMsg: severity = 4; break;
            default: break;
            }
            payload[QLatin1String("severity")] = severity;
            payload[QLatin1String("category")] = category;
            payload[QLatin1String("message")] = msg;
            payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();

            g_inLogForward = true;
            s_instance->sendToClient(buildEnvelope(QLatin1String(msg_type::kEventLog), s_instance->nextEventId(), payload));
            g_inLogForward = false;
        }

        if (s_previousHandler) {
            s_previousHandler(type, context, msg);
        }
    });
    s_logForwarderInstalled = true;
}

void HmiTcpServer::uninstallLogForwarder()
{
    if (!s_logForwarderInstalled) {
        return;
    }
    if (s_previousHandler) {
        qInstallMessageHandler(s_previousHandler);
        s_previousHandler = nullptr;
    }
    s_instance = nullptr;
    s_logForwarderInstalled = false;
}

// --- 综合检测结果推送 ---

QJsonObject HmiTcpServer::buildInspectionFinishedPayload(const flow_control::InspectionResult& result)
{
    QJsonObject payload;
    payload[QLatin1String("resultCode")] = result.resultCode;
    payload[QLatin1String("ngReasonWord0")] = result.ngReasonWord0;
    payload[QLatin1String("ngReasonWord1")] = result.ngReasonWord1;
    payload[QLatin1String("measureItemCount")] = result.measureItemCount;
    flow_control::appendInspectionMeasurementFields(payload, result.measurement);
    payload[QLatin1String("message")] = result.message;
    payload[QLatin1String("sourcePointCount")] = result.sourcePointCount;
    payload[QLatin1String("pathId")] = result.pathId;
    payload[QLatin1String("pathName")] = result.pathName;
    payload[QLatin1String("algorithm")] = result.algorithm;
    if (!result.measurement.codeValue.isEmpty()) {
        payload[QLatin1String("codeValue")] = result.measurement.codeValue;
    }
    if (result.pathId <= 0) {
        if (const auto* cfgMgr = scan_tracking::common::ConfigManager::instance()) {
            payload[QLatin1String("pathId")] = cfgMgr->activePathId();
            payload[QLatin1String("pathName")] = cfgMgr->activePathName();
            payload[QLatin1String("algorithm")] = cfgMgr->activePathAlgorithm();
        }
    }

    return payload;
}

void HmiTcpServer::publishInitialInspectionDisplay()
{
    if (!hasClient()) {
        return;
    }

    flow_control::InspectionResult idle;
    idle.resultCode = 0;
    idle.measureItemCount = 0;
    idle.sourcePointCount = 0;
    idle.message = QStringLiteral("等待检测");
    idle.measurement.qualityCode = 0;

    const QJsonObject payload = buildInspectionFinishedPayload(idle);
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventInspectionFinished), nextEventId(), payload));

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 显控连接后已推送初始检测展示帧 event.inspection.finished（等待检测）");
}

void HmiTcpServer::publishInspectionResult(const flow_control::InspectionResult& result)
{
    if (!hasClient()) {
        // TODO(hmi-demo): 无显控连接时缓存最后一帧，连接后补发
        qInfo(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 检测结果未推送（无显控连接）")
            << QStringLiteral(" resultCode=") << result.resultCode
            << QStringLiteral(" message=") << result.message;
        return;
    }

    const QJsonObject payload = buildInspectionFinishedPayload(result);
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventInspectionFinished), nextEventId(), payload));

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 检测结果已推送 event.inspection.finished")
        << QStringLiteral(" resultCode=") << result.resultCode
        << QStringLiteral(" ngWord0=") << result.ngReasonWord0
        << QStringLiteral(" measureItems=") << result.measureItemCount
        << QStringLiteral(" qualityCode=") << result.measurement.qualityCode
        << QStringLiteral(" message=") << result.message;
}

// --- 辅助发送 ---

void HmiTcpServer::sendToClient(const QJsonObject& envelope)
{
    if (!m_session) {
        return;
    }

    const QString type = envelope.value(QLatin1String("type")).toString();
    const QString msgId = envelope.value(QLatin1String("msgId")).toString();

    g_inTcpSend = true;
    if (kHmiTcpVerboseTrace) {
        const QJsonObject payload = envelope.value(QLatin1String("payload")).toObject();
        qInfo(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] → TX") << type << msgId
            << summarizeHmiTracePayload(type, payload);
    }
    m_session->sendMessage(envelope);
    g_inTcpSend = false;
}

void HmiTcpServer::sendResponse(const QString& type, const QString& msgId, bool success, const QString& message)
{
    QJsonObject payload = buildResponsePayload(success, message);
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = type;
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    sendToClient(envelope);
}

QString HmiTcpServer::nextEventId()
{
    return QStringLiteral("evt-%1").arg(m_nextEventId++);
}

}  // namespace hmi_server
}  // namespace scan_tracking
