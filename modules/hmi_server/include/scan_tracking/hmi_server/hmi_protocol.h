#pragma once

/**
 * @file hmi_protocol.h
 * @brief HMI 显控通信协议常量与 JSON 组包辅助函数
 *
 * 定义 TCP/IP JSON 协议中的所有消息类型常量，
 * 以及将内部数据结构序列化为 JSON 的工具函数。
 * 协议版本：v1.0
 */

#include <QtCore/QJsonObject>
#include <QtCore/QString>

namespace scan_tracking {
namespace hmi_server {

/// 协议版本号
inline constexpr const char* kProtocolVersion = "1.0";

/// 最大允许的单帧 JSON 字节大小（1MB）
inline constexpr quint32 kMaxFrameSize = 1048576;

/// 联调详细日志（心跳/每帧收发）；日常 false，仅 status.* 推送摘要见 pushStatusIfChanged
inline constexpr bool kHmiTcpVerboseTrace = false;

// ============================================================
// 消息类型常量定义
// ============================================================

namespace msg_type {

// --- 连接管理 ---
inline constexpr const char* kHmiHello       = "hmi.hello";
inline constexpr const char* kCoreHello      = "core.hello";        
inline constexpr const char* kHeartbeatPing  = "heartbeat.ping";
inline constexpr const char* kHeartbeatPong  = "heartbeat.pong";

// --- 状态上报（Core → Qt）---
inline constexpr const char* kStatusSystem   = "status.system";
inline constexpr const char* kStatusPlc      = "status.plc";
inline constexpr const char* kStatusCamera   = "status.camera";
inline constexpr const char* kStatusDevice   = "status.device";

// --- 流程事件（Core → Qt）---
inline constexpr const char* kEventTaskStarted       = "event.task.started";
inline constexpr const char* kEventTaskCompleted     = "event.task.completed";
inline constexpr const char* kEventTaskFailed        = "event.task.failed";
inline constexpr const char* kEventScanStarted       = "event.scan.started";
inline constexpr const char* kEventScanFinished      = "event.scan.finished";
inline constexpr const char* kEventImageCaptured     = "event.image.captured";
inline constexpr const char* kEventBundleCaptured    = "event.bundle.captured";
inline constexpr const char* kEventInspectionFinished = "event.inspection.finished";
inline constexpr const char* kEventPoseCheckFinished  = "event.pose_check.finished";
inline constexpr const char* kEventLoadGraspFinished  = "event.load_grasp.finished";
inline constexpr const char* kEventUnloadCalcFinished = "event.unload_calc.finished";
inline constexpr const char* kEventSelfCheckFinished  = "event.self_check.finished";
inline constexpr const char* kEventCodeReadFinished   = "event.code_read.finished";
inline constexpr const char* kEventResultResetFinished = "event.result_reset.finished";
inline constexpr const char* kEventPathStarted          = "event.path.started";
inline constexpr const char* kEventPathFinished         = "event.path.finished";
inline constexpr const char* kEventScanPathsAllFinished = "event.scan_paths.all_finished";
inline constexpr const char* kEventPathProgressReset    = "event.path.progress_reset";
inline constexpr const char* kEventAlarm             = "event.alarm";
inline constexpr const char* kEventLog               = "event.log";

// --- 控制命令（Qt → Core）---
inline constexpr const char* kCmdStart               = "cmd.start";
inline constexpr const char* kCmdStop                = "cmd.stop";
inline constexpr const char* kCmdReset               = "cmd.reset";
inline constexpr const char* kCmdClearAlarm          = "cmd.clear_alarm";
inline constexpr const char* kCmdGetStatus           = "cmd.get_status";
inline constexpr const char* kCmdGetConfig           = "cmd.get_config";
inline constexpr const char* kCmdTriggerScan         = "cmd.trigger_scan";
inline constexpr const char* kCmdTriggerInspection   = "cmd.trigger_inspection";
inline constexpr const char* kCmdTriggerSelfCheck    = "cmd.trigger_self_check";
inline constexpr const char* kCmdTriggerPoseCheck    = "cmd.trigger_pose_check";
inline constexpr const char* kCmdTriggerCodeRead     = "cmd.trigger_code_read";
inline constexpr const char* kCmdTriggerResultReset  = "cmd.trigger_result_reset";
inline constexpr const char* kCmdCaptureMechEye      = "cmd.capture_mech_eye";
inline constexpr const char* kCmdCaptureBundle       = "cmd.capture_bundle";
inline constexpr const char* kCmdRefreshCamera       = "cmd.refresh_camera";
inline constexpr const char* kCmdModbusConnect       = "cmd.modbus_connect";
inline constexpr const char* kCmdModbusDisconnect    = "cmd.modbus_disconnect";

/// 已废弃（原坡口配方）；仍识别并返回不支持，避免旧显控误连时无应答
inline constexpr const char* kCmdSetBevelRecipe      = "cmd.set_bevel_recipe";
inline constexpr const char* kCmdDebugTriggerInspection = "cmd.debug_trigger_inspection";

/// 显控上报监控区域人员状态：payload.alarm true=有人，false=无人（可周期重复上报）
inline constexpr const char* kCmdReportPersonZoneAlarm = "cmd.report_person_zone_alarm";

/// 显控早期拼写别名（zome → zone），与 kCmdReportPersonZoneAlarm 同义
inline constexpr const char* kCmdReportPersonZoneAlarmTypo = "cmd.report_person_zome_alarm";

}  // namespace msg_type

// ============================================================
// JSON 组包辅助函数
// ============================================================

/**
 * @brief 构建协议信封（envelope），包含 version/msgId/type/timestamp/payload
 * @param type 消息类型
 * @param msgId 消息唯一 ID
 * @param payload 消息体 JSON 对象
 * @return 完整的协议 JSON 对象
 */
QJsonObject buildEnvelope(const QString& type, const QString& msgId, const QJsonObject& payload);

/**
 * @brief 构建成功/失败响应的 payload
 * @param success 是否成功
 * @param message 描述信息
 * @return 响应 payload JSON 对象
 */
QJsonObject buildResponsePayload(bool success, const QString& message);

/**
 * @brief 将 JSON 对象序列化为带 4 字节大端长度头的帧数据
 * @param envelope 完整的协议 JSON 对象
 * @return 可直接发送的字节数组（长度头 + JSON UTF-8）
 */
QByteArray serializeFrame(const QJsonObject& envelope);

/**
 * @brief 将协议 JSON 的 UTF-8 字节转为可安全写入日志的 QString（避免 QDebug 把 QByteArray 当 Latin-1 显示乱码）
 */
QString frameJsonToLogString(const QByteArray& jsonUtf8);

/// 联调日志用 payload 单行摘要（不含大段 JSON）
QString summarizeHmiTracePayload(const QString& type, const QJsonObject& payload);

}  // namespace hmi_server
}  // namespace scan_tracking
