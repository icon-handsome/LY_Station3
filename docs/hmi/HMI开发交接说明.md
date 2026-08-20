# HMI 显控 TCP 开发交接说明

**文档版本**: v1.7  
**最后更新**: 2026-07-22  
**适用范围**: 本仓库（IPC_Station2，第二工位专用）— **仅 TCP Server 端**；麒麟 OS Qt 显控为独立 Client 工程。

> **v1.7 变更**：清理第一工位坡口 / Tracking 文案与协议样例；`cmd.get_config` 配置节改为 `scanPaths`；`event.inspection.finished` 以 `headMetrics`（含 `qualityCode`）为准；`cmd.set_bevel_recipe` 仍识别但固定失败。  
> **v1.6 变更**：`cmd.debug_trigger_inspection` 接入缓存评估 + `publishInspectionResult`（不写 PLC）。

> **新接手请先读**：本文 → [`checklists/现场联调_阶段0-1.md`](./checklists/现场联调_阶段0-1.md) → [`封头检测工位_TCP_IP显控通信协议_v1.0.md`](../protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md)

---

## 1. 架构一览

```text
┌─────────────────────┐     TCP :9900      ┌──────────────────────┐
│  Qt 显控 (麒麟 OS)   │ ◄──────────────► │  IPC Core (本仓库)    │
│  TCP Client         │  4B BE + JSON    │  HmiTcpServer         │
│  【独立仓库实现】    │                  │  modules/hmi_server  │
└─────────────────────┘                  └──────────┬───────────┘
                                                    │ 信号绑定
                    ┌───────────────────────────────┼───────────────────────────────┐
                    │ StateMachine │ Modbus │ MechEye │ VisionPipeline │ CXP A/B │
                    └───────────────────────────────────────────────────────────────┘
```

- **协议**：v1.0，帧格式 `[uint32 大端长度][UTF-8 JSON]`，信封字段 `version/msgId/type/timestamp/payload`。
- **角色**：Core = TCP Server（单客户端）；显控 = TCP Client。
- **本仓库不包含** 显控客户端代码；Qt 侧按协议自行实现（可参考 `hmi_protocol.h`）。

---

## 2. 模块与 CMake 目标（仅 Server）

| 路径 / 目标 | 职责 |
|-------------|------|
| `modules/hmi_server/` → `scan_tracking_hmi_protocol` | 协议常量、`buildEnvelope`、`serializeFrame` |
| `modules/hmi_server/` → `scan_tracking_hmi_server` | `HmiTcpServer`、`HmiSession`；监听、推送、命令分发 |
| `app/src/console_runtime.cpp` | 读 `config.ini [Hmi]` 创建 `HmiTcpServer`、`bindServiceSignals()`、`start()` |
| `config.ini` `[Hmi]` | `enabled`、`tcpPort`（默认 9900） |

**日志分类**：`hmi.server`、`hmi.session`；带 `[TCPIP]` 前缀的收发摘要。

### 2.1 检测结果推送链路

```text
Trig_Inspection（PLC）或 cmd.debug_trigger_inspection（显控）
  → evaluateStation2Inspection
  → finishInspection → HmiTcpServer::publishInspectionResult
  → TCP event.inspection.finished
```

- 显控 TCP 连接成功后推送**初始帧**（`resultCode=0`，`message="等待检测"`）。
- PLC 正式触发：写 `NG_Reason*` 并完成 Res/Ack 后推送 HMI。
- `cmd.debug_trigger_inspection`：从缓存评估并推送，**不写 PLC**。
- `cmd.set_bevel_recipe`：固定失败（已废弃）。

---

## 3. 已实现能力（Core Server）

### 3.1 传输与连接

- [x] 长度前缀帧、粘包/半包、最大 1MB
- [x] 单客户端；新连接踢旧连接
- [x] `core.hello` / `hmi.hello`
- [x] 心跳：Core 2s `heartbeat.ping`；客户端任意消息重置 6s 超时
- [x] 连接后全量 `status.*` + 周期 500ms 轮询（payload 变更去重）

### 3.2 监视面

- [x] `status.system`（含 `scanPathProgress`）/ `status.plc` / `status.camera` / `status.device`
- [x] 双 MechEye、臂/杆段进度、辅机字段与 920/921 报警
- [x] `event.scan.*`、`event.path.*`、`event.bundle.captured`、`event.image.captured`
- [x] `event.alarm`、业务 `event.*.finished`（绑定 StateMachine）

### 3.3 控制与调试命令

- [x] `cmd.start` / `cmd.stop` / `cmd.reset` / `cmd.clear_alarm`
- [x] `cmd.get_status` / `cmd.get_config`（含 `scanPaths`）
- [x] `cmd.modbus_connect` / `cmd.modbus_disconnect`
- [x] `cmd.capture_mech_eye` / `cmd.capture_bundle` / `cmd.refresh_camera`
- [x] `cmd.debug_trigger_inspection`
- [x] `cmd.set_bevel_recipe`（废弃：固定失败）
- [x] `cmd.report_person_zone_alarm`

### 3.4 检测测量结构化

- [x] `event.inspection.finished`：`resultCode`、`ngReasonWord*`、`pathId`/`algorithm`、`headMetrics.*`、`message`

---

## 4. 刻意未开放 / 待办（Core 侧）

| 项 | 状态 | 说明 |
|----|------|------|
| `cmd.trigger_scan` / `cmd.trigger_inspection` 等 | **拒绝** | 须 PLC→状态机触发（防撞机） |
| `cmd.set_bevel_recipe` | **废弃** | 固定失败应答 |
| `event.task.*` | 未实现 | 协议常量已有 |
| `event.log` | 默认关闭 | `kForwardQtLogsToHmi=false` |
| 无显控连接时补发最后一帧检测结果 | 未实现 | 可选增强 |
| Qt 显控 Client + UI | **外部仓库** | 本仓库不维护客户端 |

---

## 5. 关键代码入口

| 场景 | 文件 |
|------|------|
| 服务启动 | `app/src/console_runtime.cpp` |
| 命令分发 / 状态推送 / `publishInspectionResult` | `modules/hmi_server/src/hmi_tcp_server.cpp` |
| 帧组包 / 协议常量 | `modules/hmi_server/src/hmi_protocol.cpp`、`hmi_protocol.h` |

## 6. 本地快速验证

1. `config.ini [Hmi] enabled=true`，`tcpPort=9900`
2. 启动 Core，确认 HMI TCP 监听日志
3. 由 Qt 显控或任意遵守协议的 TCP 客户端连接并联调

## 7. Qt 显控侧（不在本仓库）

1. 按协议实现长度头 + JSON 解帧与 `type` 分发
2. 绑定 `headMetrics.qualityCode` 与第二工位测量字段；勿再依赖坡口键 / 顶层 `quality_code` / `cmd.get_config.tracking`
3. 可参考本仓库 `scan_tracking_hmi_protocol` 的常量与 `serializeFrame` 语义（拷贝即可，勿在本仓加 Client 目标）

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| [封头检测工位_TCP_IP显控通信协议_v1.0.md](../protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md) | 消息与 payload（文件名历史遗留，内容已按第二工位修订） |
| [现场联调_阶段0-1.md](./checklists/现场联调_阶段0-1.md) | 联调清单 |
| [第二工位骨架删减说明.md](../guides/第二工位骨架删减说明.md) | 本仓 HMI 保留范围 |
