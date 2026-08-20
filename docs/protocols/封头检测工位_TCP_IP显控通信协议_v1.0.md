# 第二工位 IPC-Qt 显控通信协议

**版本**：v1.0（第二工位修订：去掉坡口/Tracking 遗留说明，对齐当前 Core 推送字段）  
**适用范围**：`IPC_Station2` 核心控制程序（Windows）与 Qt 显控界面（麒麟 OS）的 TCP/IP 通信。

---

## 1. 通信基础约定

### 1.1 网络架构
- **TCP Server**：核心控制程序（Core）。
- **TCP Client**：Qt 显控程序（Qt）。单客户端连接，不支持多个显控端同时连接。
- **异常处理**：
  - 心跳机制：双方周期发送心跳包（建议 2000ms），超过 6 秒未收到心跳判断为断连。
  - 断连行为：Qt 断连时 Core 不自动暂停，保持继续运行。

### 1.2 报文格式
TCP 是流式协议，为解决粘包和半包问题，采用长度前缀的帧格式：
```text
[4 字节大端长度头] + [JSON 正文 UTF-8]
```
- **长度头**：`uint32_t`，大端网络字节序（Big-Endian），表示后续 JSON 正文的字节数。
- **正文**：不包含大点云和 2D 图像原始数据，仅传输数据摘要。

### 1.3 JSON 基本结构
所有通信的 JSON 正文统一格式如下：
```json
{
  "version": "1.0",
  "msgId": "uuid-或自增序列号",
  "type": "消息类型",
  "timestamp": 1710000000000,
  "payload": {}
}
```
- `version`：协议版本，固定 `"1.0"`。
- `msgId`：请求-响应匹配的唯一标识。
- `type`：消息类型，见后续定义。
- `timestamp`：发送时的 Unix 毫秒时间戳。
- `payload`：实际数据，依 `type` 变化。

### 1.4 消息模型
1. **request**：Qt 主动发起请求或发送控制命令。
2. **response**：Core 对 request 的应答，`msgId` 必须与请求一致。
3. **event**：Core 主动上报状态、结果、报警、日志等，Core 自行生成 `msgId`。

---

## 2. 状态上报与事件 (Event)

由 Core 主动发给 Qt，频率根据字段要求不同。

### 2.1 心跳
- `type`: `heartbeat.ping` (发送方), `heartbeat.pong` (接收方响应)
- `payload`: `{}`

### 2.2 系统运行状态 (`status.system`)
- **频率**：状态变更上报，或 500ms 周期。
- **payload 字段**：
  - `ipcState` (int): 0=未初始化, 1=初始化中, 2=就绪, 3=忙碌, 4=暂停, 5=故障
  - `appState` (string): "Init", "Ready", "Scanning", "Error"
  - `stage` (int): 当前工艺阶段（含 `11`=伸缩杆扫描 TelescopicScan）
  - `alarmLevel` (int): 0=无, 1=提示, 2=黄警, 3=红警
  - `alarmCode`, `warnCode` (int)
  - `ipcReady` (int): 0/1
  - `progress` (int): 0~100
  - `stationId` / `stationName` / `workMode`(string，工位 profile) / `enabledTriggers`
  - `scanPathProgress` (object)：当前路径基础显示，见 §2.5 / `docs/hmi/路径状态交互指令.txt`

### 2.3 PLC 状态 (`status.plc`)
- **频率**：500ms 周期。
- **payload 字段**：
  - `plcHeartbeat` (int)
  - `plcSystemState` (int): 0=待机, 1=自动, 2=暂停, 3=报警停机, 4=手动
  - `workMode` (int): 0=空闲, 1=上料, 2=扫描, 3=检测, 4=下料
  - `flowEnable` (int): 0/1
  - `safetyWord` (int): 位域字典
  - `taskId` (int), `productType` (int), `recipeId` (int)
  - `armScanSegmentIndex` / `telescopicScanSegmentIndex` / `scanSegmentIndex`(兼容，等同臂段号)
  - `scanSegmentTotal`、`activePathId` / `activePathName` / `activePathAlgorithm`
  - `armPointCount` / `telescopicPointCount`
  - `robotStatusWord` (int): 埃斯顿机械臂状态字（PLC 转发 Robot Modbus 40004，位定义见 IPC-PLC 协议 §8.1.1）
  - `telescopicRodStatus` (int): 伸缩杆状态，0=待机, 1=运行, 2=故障（PLC 40041）
  - `rollerSetFreqHz` (int): 滚轮设定频率 Hz（PLC 40042）
  - `rollerRunFreqHz` (int): 滚轮运行频率 Hz（PLC 40043）
  - `electromagnetStatus` (int): 电磁吸盘状态，0=退磁, 1=充磁, 2=报警（PLC 40044）
  - `estopButtonStatus` (int): 急停按钮，0=断开(未按下), 1=按下（PLC 40045）
  - `modbusConnected` (bool)
  - `stationId` / `stationName` / `stationWorkMode`

> **辅机字段**：无 PLC 数据时缺省为 0。`telescopicRodStatus` 或 `electromagnetStatus` 变为 **2** 时，Core 向显控推送 `event.alarm`（`level=2`，`code` 920/921，见 §2.8）。

### 2.4 相机与设备状态 (`status.camera` / `status.device`)
- **频率**：
  - Core 每 **500ms** 轮询一次是否需推送，但 **仅当 JSON payload 与上次下发不一致时才真正发送**（变更去重，稳态下不会每 500ms 刷一条）。
  - 相机/流水线/梅卡等 **连接态或 `state` 变化** 时立即尝试推送，同样受 payload 去重约束（海康仅 `connected` 变化才触发 `status.camera` 内容变化，采图过程中的文字状态不会刷屏）。
  - 新客户端接入时 **强制全量** 各推送一次（含 `status.camera`）。
- **显控侧预期**：稳态监视下 `status.camera` 很少出现；连接瞬间可能连收数条（双 MechEye / hik / pipeline 分别就绪时）；不应出现无变化的周期性刷屏。
- **camera payload**:
  - `mechEyeTelescopic` / `mechEyeArm`: `{ roleName, state(int), connected(bool) }`
  - `mechEye`: 兼容别名，等同伸缩杆 MechEye
  - `hikA`, `hikB`, `hikC`: `{ roleName, connected(bool) }`
  - `hikCTelescopic` / `hikCArm`: `{ ipAddress, connected(bool) }`（智能相机分组）
  - `pipeline`: `{ state(int) }`
- **device payload**:
  - `onlineWord0`, `faultWord0` (int) 位域字典（`online` 与 `fault` 按位对齐，便于显控做设备条）

**`status.device` 位定义（word0，低位 Bit0）**

| Bit | `onlineWord0` | `faultWord0` |
|-----|---------------|--------------|
| 0 | IPC Core 进程在线 | IPC 故障（`ipcState=Fault` / `appState=Error` / 红警 `alarmLevel≥3`） |
| 1 | HMI 客户端已连接 | （保留） |
| 2 | Mech-Eye 可用（非 Idle/Error；臂或杆任一） | Mech-Eye `Error` |
| 3 | 视觉流水线 `Ready` | 视觉流水线 `Error` |
| 4 | CXP 双目 A/B 或海康 C 任一台已连接 | CXP A/B 与海康 C 均未连接（服务已配置时） |
| 5 | 扫描流水线已启动（`VisionPipeline::isStarted`） | （保留） |
| 6 | Modbus 已连接 | Modbus 未连接 |
| 7 | （保留） | 黄警 `alarmLevel≥2` |

### 2.5 流程事件 (`event.scan.*` / `event.task.*`)
- `event.scan.started`: `{ segmentIndex, taskId, pathId?, pathName?, purpose? }`
- `event.scan.finished`: `{ segmentIndex, resultCode, imageCount, cloudFrameCount, pathId?, pathName? }`
- `event.image.captured`: `{ requestId, cameraKey, pointCount, width, height, elapsedMs, errorCode }` (不传原图)
- `event.bundle.captured`: `{ segmentIndex, taskId, mechOk, hikAOk, hikBOk }`
- `event.path.started` / `event.path.finished`：路径级进度（见 `docs/hmi/路径状态交互指令.txt`）
- `event.scan_paths.all_finished` / `event.path.progress_reset`
- `event.task.*`：协议保留，当前 Core 未推送

`status.system` 另含 `scanPathProgress`：`currentPathId` / `currentPathName` / `enabledPathIds` / `completedPathIds` / `pathCount` / `allPathsComplete`。

### 2.6 检测结果 (`event.inspection.finished`)

> 第二工位测量量放在嵌套对象 `headMetrics`（键名沿用历史，语义为筒体/焊缝等综合指标，**不是**坡口 Po_Kou）。

- **产生时机**：
  - **正式**：PLC `Trig_Inspection` → `InspectionHandler` → `finishInspection` → `publishInspectionResult`
  - **调试**：显控 `cmd.debug_trigger_inspection` → `evaluateCachedInspection` → `publishInspectionResult`（不写 PLC）
- **推送策略**：`resultCode=1`（OK）、`2`（NG）、`3`（数据/缓存异常）、`6`（流程异常）均可推送；显控须处理失败场景。
- **连接初始化**：显控 TCP 接入成功后，Core **一次性**推送 `event.inspection.finished`，`resultCode=0`，`headMetrics.qualityCode=0`，`message="等待检测"`。
- **payload 顶层字段**：
  - `resultCode` (int): **0=尚未检测/连接占位**，1=OK, 2=NG, 3=缓存/数据异常, 6=流程异常
  - `ngReasonWord0`, `ngReasonWord1` (int)
  - `measureItemCount` (int)
  - `sourcePointCount` (int)
  - `pathId` / `pathName` / `algorithm` (string)
  - `codeValue` (string，可选)
  - `message` (string)
  - `headMetrics` (object)：见下

- **`headMetrics` 主要字段**（按算法路径填充，未测项可为 0）：
  - `qualityCode` (int): 1=通过，2=不通过/无效
  - 焊缝：`mismatchMm` / `reinforcementMm` / `angularityMm` / `includedAngleDeg` / `leftUndercutMm` / `rightUndercutMm` / `maxUndercutMm` / `measuredSegmentCount`
  - 厚度/内表面：`thicknessMm` / `thicknessPairCount` / `thicknessSuccessCount` / `innerDiameterMm` / `innerCircumferenceMm` / `innerRoundness` / …
  - 长度容积：`lengthMm` / `volumeLiters` / `volumeRadiusMm` / `fittedOuterRadiusMm`
  - 部分字段同时带 snake_case 别名（如 `thickness_mm`），便于旧绑定迁移

### 2.7 其他检测校验完成 (`event.xxx.finished`)
- `event.pose_check.finished`: `{ success, resultCode, poseDeviationMm, rt[16], message }`
- `event.load_grasp.finished`: `{ resultCode, x, y, z, rx, ry, rz }`
- `event.unload_calc.finished`: `{ resultCode, x, y, z, rx, ry, rz }`
- `event.self_check.finished`: `{ resultCode, failWord0 }`
- `event.code_read.finished`: `{ resultCode, codeValue(string) }`
- `event.result_reset.finished`: `{ resultCode }`

### 2.8 报警与日志
- `event.alarm`: `{ level, code, message, timestamp }` (单向发送，无需回执)
- **辅机 PLC 报警 code（920 段，与 Modbus 900 段、相机 910 段区分）**：
  - `920`：`telescopicRodStatus` 变为 2（伸缩杆故障），`level=2`
  - `921`：`electromagnetStatus` 变为 2（电磁吸盘报警），`level=2`
  - 边沿触发：仅在由非 2 变为 2 时推送一次；新客户端接入后同步缓存，不重复推送当前故障态
- `event.log`: `{ severity, category, message, file, line, timestamp }`

---

## 3. Qt 控制命令 (Request / Response)

Qt 发送 request（附带不重复的 `msgId`），Core 执行后返回对应 `msgId` 的 response，response `payload` 基础结构包含：`{ "success": true/false, "message": "描述" }`，部分命令会附加额外字段。

| type | request payload | response payload 附加字段 | 说明 |
|---|---|---|---|
| `cmd.start` | `{}` | - | 启动状态机 |
| `cmd.stop` | `{}` | - | 停止状态机 |
| `cmd.reset` | `{}` | - | 重置状态 |
| `cmd.clear_alarm` | `{}` | - | 清除当前报警记录 |
| `cmd.get_status` | `{}` | `system`, `plc`, `camera`, `device` 全量状态对象 | 主动拉取全量状态 |
| `cmd.get_config` | `{ "section": "..." }` | 全量 JSON：`app`/`logger`/`modbus`/`camera`/`vision`/`flowControl`/`scanPaths`/`hmi`（无坡口配方） | 获取 Core 侧配置 |
| `cmd.set_bevel_recipe` | （任意） | - | **废弃**：固定返回失败（第二工位无坡口配方） |
| `cmd.trigger_scan` | `{ "segmentIndex": 1, "taskId": 123 }` | - | 触发单段扫描（**Core 拒绝**，须 PLC） |
| `cmd.trigger_inspection` | `{ "taskId": 123 }` | - | 触发综合检测（**Core 拒绝**，须 PLC） |
| `cmd.debug_trigger_inspection` | `{}` | - | 从段缓存评估并推送 `event.inspection.finished`（不写 PLC） |
| `cmd.trigger_self_check` | `{}` | - | 显控触发自检；**Core 已接收并应答 success**，完整流程待完善 |
| `cmd.trigger_pose_check` | `{}` | - | **Core 拒绝**，须 PLC |
| `cmd.trigger_code_read` | `{}` | - | **Core 拒绝**，须 PLC |
| `cmd.trigger_result_reset`| `{}` | - | **Core 拒绝**，须 PLC |
| `cmd.capture_mech_eye` | `{ "cameraKey":"...", "mode":0, "timeoutMs":5000 }` | `"requestId": 123` | 单相机独立采图 |
| `cmd.capture_bundle` | `{ "segmentIndex":1, "taskId": 123 }` | `"requestId": 123` | 触发多相机集成采集 |
| `cmd.refresh_camera` | `{}` | - | 刷新相机连接状态 |
| `cmd.modbus_connect` | `{}` | - | 重连 PLC Modbus |
| `cmd.modbus_disconnect` | `{}` | - | 断开 PLC Modbus |

> **备注**：不需要支持 `cmd.set_config`（热修改配置），不涉及直接控制 PLC 寄存器的命令（Qt 不直接控制 PLC）。  
> **`cmd.trigger_*` 与 `cmd.debug_trigger_inspection` 区别**：除 `cmd.trigger_self_check`（仅接收应答，执行待完善）外，其余 `cmd.trigger_*` 一律拒绝（防撞机）；`cmd.debug_trigger_inspection` 为联调入口，不写 PLC。

---

## 4. 示例

### 4.1 触发检测被拒绝（须走 PLC）
**Qt 请求：**
```json
{
  "version": "1.0",
  "msgId": "req-101",
  "type": "cmd.trigger_inspection",
  "timestamp": 1710000000000,
  "payload": {
    "taskId": 123
  }
}
```
**Core 响应：**
```json
{
  "version": "1.0",
  "msgId": "req-101",
  "type": "cmd.trigger_inspection",
  "timestamp": 1710000000050,
  "payload": {
    "success": false,
    "message": "直接触发未实现，请使用 PLC"
  }
}
```

### 4.2 检测完成事件上报
**Core 上报：**
```json
{
  "version": "1.0",
  "msgId": "evt-200",
  "type": "event.inspection.finished",
  "timestamp": 1710000005000,
  "payload": {
    "resultCode": 1,
    "ngReasonWord0": 0,
    "ngReasonWord1": 0,
    "measureItemCount": 1,
    "sourcePointCount": 125000,
    "pathId": 1,
    "pathName": "straight_weld",
    "algorithm": "weld_section",
    "message": "焊缝检测通过",
    "headMetrics": {
      "qualityCode": 1,
      "mismatchMm": 0.12,
      "reinforcementMm": 1.05,
      "maxUndercutMm": 0.3,
      "measuredSegmentCount": 4,
      "thicknessMm": 0,
      "lengthMm": 0,
      "volumeLiters": 0
    }
  }
}
```
