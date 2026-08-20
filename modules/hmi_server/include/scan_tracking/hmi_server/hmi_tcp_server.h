#pragma once

/**
 * @file hmi_tcp_server.h
 * @brief HMI TCP 服务端，负责与 Qt 显控界面的通信
 *
 * 本模块是核心控制程序与 Qt 显控界面之间的通信桥梁，主要功能：
 * 1. 监听 TCP 端口，接受单个 Qt 客户端连接
 * 2. 周期性推送系统状态（IPC/PLC/相机/设备）
 * 3. 转发业务事件（扫描/检测/报警等）
 * 4. 接收并处理 Qt 端的控制命令
 *
 * 注意：本模块不直接控制 PLC，所有命令都通过 StateMachine 间接调度。
 */

#include <QtCore/QObject>
#include <QtCore/QHash>
#include <QtCore/QTimer>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QJsonObject>

#include "scan_tracking/flow_control/inspection_types.h"

QT_BEGIN_NAMESPACE
class QTcpServer;
QT_END_NAMESPACE

namespace scan_tracking {
namespace modbus { class ModbusService; }
namespace mech_eye { class MechEyeService; }
namespace flow_control { class StateMachine; }
namespace vision {
class HikCxpCameraService;
class HikCameraService;
class HikCameraCController;
class VisionPipelineService;
}
namespace hmi_server {

class HmiSession;

/// 单路 status.* 去重缓存（按 payload 相等抑制重复下发，避免空 QJsonObject 与“未缓存”混淆）
struct HmiStatusPushCache {
    QJsonObject payload;
    bool isValid = false;
};

class HmiTcpServer : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param port 监听端口号
     * @param parent 父对象
     */
    explicit HmiTcpServer(int port = 9900, QObject* parent = nullptr);
    ~HmiTcpServer() override;

    /// 绑定各业务模块信号（依赖注入完成后调用一次，勿在 start() 里重复 connect）
    void bindServiceSignals();

    /// 启动 TCP 服务器
    bool start();

    /// 停止 TCP 服务器
    void stop();

    /// 检查服务器是否正在监听
    bool isListening() const;

    /// 检查是否有客户端已连接
    bool hasClient() const;

    /**
     * @brief 主动推送综合检测结果到 Qt 显控（`event.inspection.finished`）
     *
     * 成功与失败均推送。无客户端时当前不缓存；多路径结果字段见 InspectionResult / headMetrics。
     */
    void publishInspectionResult(const flow_control::InspectionResult& result);

    // --- 设置服务依赖（在 start() 之前调用） ---
    void setStateMachine(flow_control::StateMachine* sm);
    void setModbusService(modbus::ModbusService* svc);
    void setMechEyeService(mech_eye::MechEyeService* svc);
    void setMechEyeServices(mech_eye::MechEyeService* telescopic, mech_eye::MechEyeService* arm);
    void setVisionPipelineService(vision::VisionPipelineService* svc);
    void setHikCameraServices(vision::HikCxpCameraService* hikA, vision::HikCxpCameraService* hikB,
                              vision::HikCameraService* hikC = nullptr);
    void setHikCameraCController(vision::HikCameraCController* controller);

private slots:
    /// 处理新的客户端连接
    void onNewConnection();

    /// 处理客户端断开
    void onSessionDisconnected();

    /// 处理客户端心跳超时
    void onSessionHeartbeatTimeout();

    /// 处理接收到的客户端消息
    void onMessageReceived(const QJsonObject& message);

    /// 周期性推送状态
    void onStatusPushTimer();

    /// 周期性发送心跳
    void onHeartbeatTimer();

private:
    // --- 命令处理函数 ---
    
    /// 消息处理函数指针类型定义
    using MessageHandler = void (HmiTcpServer::*)(const QJsonObject&);
    
    /// 初始化消息处理函数映射表
    void initializeMessageHandlers();
    
    /// 处理 Qt 端的 hmi.hello 握手请求
    void handleHmiHello(const QJsonObject& message);
    
    /// 处理 Qt 端发来的心跳 ping 包，返回 pong
    void handleHeartbeatPing(const QJsonObject& message);

    void handleHeartbeatPong(const QJsonObject& message);
    
    /// 处理启动指令，触发核心状态机进入启动状态
    void handleCmdStart(const QJsonObject& message);
    
    /// 处理停止指令，触发核心状态机停止
    void handleCmdStop(const QJsonObject& message);
    
    /// 处理复位指令，重置核心系统异常状态
    void handleCmdReset(const QJsonObject& message);
    
    /// 处理清除报警指令，清空当前系统的报警码
    void handleCmdClearAlarm(const QJsonObject& message);
    
    /// 处理获取全量状态指令，立刻推送系统、PLC、相机等全量状态
    void handleCmdGetStatus(const QJsonObject& message);
    
    /// 处理获取配置指令 (暂未开放热更配置，返回空)
    void handleCmdGetConfig(const QJsonObject& message);
    
    /// 处理连接 Modbus 指令，手动重连 PLC
    void handleCmdModbusConnect(const QJsonObject& message);
    
    /// 处理断开 Modbus 指令，手动断开 PLC
    void handleCmdModbusDisconnect(const QJsonObject& message);
    
    /// 处理刷新相机连接指令
    void handleCmdRefreshCamera(const QJsonObject& message);
    
    /// 处理触发扫描指令 (安全限制：拦截 Qt 端的直接触发，防止撞机，必须由 PLC 触发)
    void handleCmdTriggerScan(const QJsonObject& message);
    
    /// 处理触发综合检测指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerInspection(const QJsonObject& message);
    
    /// 处理触发自检指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerSelfCheck(const QJsonObject& message);
    
    /// 处理触发位姿校验指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerPoseCheck(const QJsonObject& message);
    
    /// 处理触发扫码指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerCodeRead(const QJsonObject& message);
    
    /// 处理触发结果复位指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerResultReset(const QJsonObject& message);

    /// 显控上报监控区域有无人员（经 StateMachine 写 IPC_SafetyAction_Word 停 PLC）
    void handleCmdReportPersonZoneAlarm(const QJsonObject& message);
    
    /// 处理单独通过 Mech-Eye 进行采图的指令（多用于独立标定或调试测试）
    void handleCmdCaptureMechEye(const QJsonObject& message);
    
    /// 处理发起多相机联合采集的点云/图像 Bundle 指令
    void handleCmdCaptureBundle(const QJsonObject& message);

    /// 废弃命令：一律失败应答（兼容旧显控）
    void handleCmdSetBevelRecipe(const QJsonObject& message);
    void handleCmdDebugTriggerInspection(const QJsonObject& message);


    // --- 状态推送函数 ---
    
    /// 向 Qt 端周期性推送系统整体运行状态（如 IPCState、报警级别、整体进度等）
    void pushSystemStatus();

    QJsonObject buildSystemStatusPayload() const;
    QJsonObject buildScanPathProgressPayload() const;
    
    /// 向 Qt 端周期性推送 PLC 连接状态及内部部分寄存器运行状态
    void pushPlcStatus();

    QJsonObject buildPlcStatusPayload() const;
    
    /// 向 Qt 端周期性推送各相机的连接状态与运行阶段
    void pushCameraStatus();

    QJsonObject buildCameraStatusPayload() const;

    /// 梅卡 connected 语义（与 buildCameraStatusPayload 一致，采图过程仍视为已连接）
    bool mechEyeServiceConnected(const mech_eye::MechEyeService* service) const;
    bool hikCameraCConnected(const QString& cameraIp) const;
    bool hikCameraCConnected() const;
    mech_eye::MechEyeService* resolveMechEyeService(const QString& cameraKey) const;

    /// 同步相机连/断边沿缓存（新客户端接入或拉全量状态后调用，不发送 alarm）
    void syncCameraConnectivityCache();

    /// 检测 connected 边沿并向显控发送 event.alarm（仅连/断，不含采图过程 state 变化）
    void checkCameraConnectivityEdges();

    void emitCameraConnectivityAlarm(const QString& deviceLabel, bool connected, int code);

    /// 检测辅机状态字变为 2（故障/报警）时向显控推送 event.alarm
    void checkPlcAuxDeviceAlarms(const QVector<quint16>& commandBlock);

    void emitPlcAuxDeviceAlarm(const QString& message, int code, int level);

    /// 同步辅机 alarm 边沿缓存（新客户端接入后调用，不发送 alarm）
    void syncPlcAuxDeviceAlarmCache(const QVector<quint16>& commandBlock);
    
    /// 向 Qt 端周期性推送设备在线状态字和故障状态字（对应协议 status.device）
    void pushDeviceStatus();

    QJsonObject buildDeviceStatusPayload() const;

    /// payload 与缓存一致则不下发；forcePush 用于新客户端接入后的全量同步
    bool pushStatusIfChanged(const QString& type, const QJsonObject& payload, HmiStatusPushCache& slot,
                             bool forcePush = false);

    /// 清空去重缓存（仅在新连接 / 断开时调用）
    void invalidateStatusPushCache();

    /// 向当前客户端全量推送四类 status.*（强制下发，不受去重影响）
    void pushAllStatusToClient();

    /// 用当前构建结果刷新去重缓存（cmd.getStatus 等已主动下发状态后调用）
    void syncStatusPushCacheFromCurrent();


    // --- 事件上报连接 ---
    
    /// 断开 bindServiceSignals 建立的连接（stop 时调用，防止重复 bind 崩溃）
    void disconnectServiceSignals();

    /// 绑定状态机发出的各种业务及异常信号，组装 JSON 转发到 Qt 端
    void connectStateMachineSignals();
    
    /// 绑定 Modbus 底层通信抛出的断连、超时等信号，组装 JSON 转发到 Qt 端
    void connectModbusSignals();
    
    /// 绑定 MechEye 3D 相机抛出的采集完成或故障信号
    void connectMechEyeSignals();
    
    /// 绑定视觉流水线抛出的多相机 Bundle 采集完成信号
    void connectVisionPipelineSignals();

    /// 状态/设备变化时立即刷新 status.*（阶段 1 显控监视）
    void connectStatusRefreshSignals();
    
    // TODO(hmi): 远程 event.log 默认未启用（hmi_tcp_server.cpp 中 kForwardQtLogsToHmi=false）。
    /// 安装全局 Qt 日志拦截器，将非 HMI 模块的 Warning+ 转发为 event.log（显控有日志页时再开）
    void installLogForwarder();
    
    /// 卸载全局 Qt 日志拦截器，恢复原始日志处理器
    void uninstallLogForwarder();


    // --- 发送辅助函数 ---
    
    /// 将封装好的完整 JSON 信封通过 TCP Session 发送给客户端
    /// @param envelope 带有 version, msgId, type, timestamp, payload 的完整 JSON 对象
    void sendToClient(const QJsonObject& envelope);
    
    /// 辅助构造并发送标准格式的响应报文 (Response) 给客户端
    /// @param type    响应对应的命令 type
    /// @param msgId   对应请求传入的 msgId
    /// @param success 指令是否执行成功
    /// @param message 附带的结果描述信息
    void sendResponse(const QString& type, const QString& msgId, bool success, const QString& message);
    
    /// 生成递增的唯一事件 ID (如: evt-1, evt-2)，用于 Core 主动上报的 event
    QString nextEventId();

    /// 由 InspectionResult 组装 event.inspection.finished 的 payload
    static QJsonObject buildInspectionFinishedPayload(const flow_control::InspectionResult& result);

    /// 显控连接后一次性推送全零检测展示帧（resultCode=0，便于 UI 初始化绑定）
    void publishInitialInspectionDisplay();

    QTcpServer* m_tcpServer = nullptr;      ///< Qt TCP 服务器
    HmiSession* m_session = nullptr;        ///< 当前客户端会话（单客户端模式）
    int m_port = 9900;                      ///< 监听端口
    QTimer* m_statusPushTimer = nullptr;    ///< 状态推送定时器
    QTimer* m_heartbeatTimer = nullptr;     ///< 心跳发送定时器
    quint64 m_nextEventId = 1;             ///< 事件 ID 自增序号

    HmiStatusPushCache m_systemStatusCache;  ///< status.system 去重缓存
    HmiStatusPushCache m_plcStatusCache;     ///< status.plc 去重缓存
    HmiStatusPushCache m_cameraStatusCache;  ///< status.camera 去重缓存
    HmiStatusPushCache m_deviceStatusCache;  ///< status.device 去重缓存

    /// 相机 connected 边沿缓存（与 status.camera 判定一致，用于连/断 alarm）
    struct CameraConnectivityCache {
        bool mechEyeTelescopic = false;
        bool mechEyeArm = false;
        bool hikA = false;
        bool hikB = false;
        bool hikCTelescopic = false;
        bool hikCArm = false;
        bool valid = false;
    };
    CameraConnectivityCache m_cameraConnectivityCache;

    /// 辅机状态 alarm 边沿缓存（TelescopicRod / Electromagnet status=2）
    struct PlcAuxDeviceAlarmCache {
        int telescopicRodStatus = -1;
        int electromagnetStatus = -1;
        bool valid = false;
    };
    PlcAuxDeviceAlarmCache m_plcAuxDeviceAlarmCache;

    bool m_personZoneAlarm = false;          ///< 上次显控上报的人员区域 alarm 值
    bool m_personZoneAlarmCacheValid = false; ///< 是否已收到过至少一次人员区域上报

    bool m_serviceSignalsBound = false;      ///< 是否已 bindServiceSignals

    // --- 服务依赖指针 ---
    flow_control::StateMachine* m_stateMachine = nullptr;
    modbus::ModbusService* m_modbusService = nullptr;
    mech_eye::MechEyeService* m_mechEyeTelescopic = nullptr;
    mech_eye::MechEyeService* m_mechEyeArm = nullptr;
    vision::VisionPipelineService* m_visionPipeline = nullptr;
    vision::HikCxpCameraService* m_hikCameraA = nullptr;
    vision::HikCxpCameraService* m_hikCameraB = nullptr;
    vision::HikCameraService* m_hikCameraC = nullptr;
    vision::HikCameraCController* m_hikCameraCController = nullptr;
    
    /// 消息类型到处理函数的映射表（用于快速分发消息）
    QHash<QString, MessageHandler> m_messageHandlers;
    
    /// 全局日志拦截器回调需要访问当前实例，使用静态指针保存（单实例模式）
    static HmiTcpServer* s_instance;
    
    /// 安装日志拦截器之前保存的原始日志处理器，卸载时恢复
    static QtMessageHandler s_previousHandler;

    /// 防止重复 install/uninstall 破坏 qInstallMessageHandler 链
    static bool s_logForwarderInstalled;
};

}  // namespace hmi_server
}  // namespace scan_tracking
