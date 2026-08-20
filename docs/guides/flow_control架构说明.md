# flow_control 模块架构说明

> **文档版本**：v1.0  
> **更新日期**：2026-06-16  
> **仓库**：`IPC_Station3`（第三工位）  
> **读者**：IPC 开发 / 接手 AI Agent

---

## 1. 设计目标

第三工位沿用通用骨架，`StateMachine` 不再承载全部业务逻辑，而是：

1. **Modbus 编排**：轮询 PLC、触发分发、Ack/Res、超时与故障
2. **异步回调**：视觉流水线 `bundleCaptureFinished`、Mech-Eye 致命错误
3. **宿主能力**：通过 `PlcTaskHost` 向 Handler 暴露 PLC 写入、段缓存、HMI 事件

各 `Trig_*` 的**同步业务**写在 `handlers/*.cpp`，新增触发器时不必再改 `state_machine.h` 的 private 区。

---

## 2. 组件关系

```text
PLC 命令块
    ↓ poll / processTrigger
StateMachine::executeActiveTask()
    ↓ TaskHandlerRegistry
ITaskHandler::execute(TaskHandlerContext)
    ↓ ctx.host (PlcTaskHost&)
    ├─ completeActiveTask / write* / finishInspection …
    └─ notify* → Qt signals → HmiTcpServer

VisionPipelineService::bundleCaptureFinished
    ↓ (仍由 StateMachine 槽函数处理)
StateMachine::onBundleCaptureFinished → ScanSegmentCache 落盘 → completeScanSegmentCapture
```

---

## 3. 核心类型

| 类型 | 路径 | 职责 |
|------|------|------|
| `StateMachine` | `include/.../state_machine.h` | QObject + PlcTaskHost 实现；对外 status/HMI 查询 |
| `PlcTaskHost` | `include/.../plc_task_host.h` | Handler 可调用的虚接口（PLC、检测、事件） |
| `TaskHandlerContext` | `include/.../task_handler_context.h` | `{ host, commandBlock, activeTask }` |
| `ITaskHandler` | `include/.../handlers/itask_handler.h` | 各触发器 `execute()` 入口 |
| `TaskHandlerRegistry` | `task_handler_registry.*` | 按 `trigOffset` 查找 Handler |
| `ScanSegmentCache` | `scan_segment_cache.*` | 段内存缓存 + `output/run_*` 落盘 |
| `evaluateStation3Inspection` | `station3_inspection.*` | Inspection 占位算法（缓存校验） |

内部工具（**模块内**）：`include/.../detail/state_machine_internal.h` — 日志常量、`parsePoseSource`、`countBundleFrames` 等。

---

## 4. StateMachine 源文件拆分

| 文件 | 内容 |
|------|------|
| `state_machine.cpp` | 构造/启停、`executeActiveTask`、PlcTaskHost 服务访问与 `notify*` |
| `state_machine_modbus.cpp` | PLC 轮询、触发、`completeActiveTask`、故障 |
| `state_machine_plc_io.cpp` | 寄存器写入、位姿占位、自检/复位寄存器清理 |
| `state_machine_scan.cpp` | 段采集异步完成、`completeScanSegmentCapture` |
| `state_machine_inspection.cpp` | `finishInspection`、`evaluateInspectionForActiveTask` |
| `state_machine_helpers.cpp` | `LOG_FLOW`、PLC 日志格式化、工具函数 |

---

## 5. Handler 与触发器

| Handler | 文件 | 实现状态 |
|---------|------|----------|
| `ScanSegmentHandler` | `scan_segment_handler.cpp` | **真实采集**（发起 bundle，异步完成在 StateMachine） |
| `InspectionHandler` | `inspection_handler.cpp` | **缓存校验占位** → `finishInspection` |
| `LoadGraspHandler` | `load_grasp_handler.cpp` | 位姿占位（环境变量 / 模拟值） |
| `UnloadCalcHandler` | `unload_calc_handler.cpp` | 同上 |
| `PoseCheckHandler` | `pose_check_handler.cpp` | 占位 OK（LB/CXP 链路保留） |
| `SelfCheckHandler` | `self_check_handler.cpp` | Modbus + MechEye + Vision 就绪检查 |
| `StationMaterialCheckHandler` | `station_material_check_handler.cpp` | 占位 OK |
| `CodeReadHandler` | `code_read_handler.cpp` | 海康 C 编号 OCR（`NumberRecognition`） |
| `ResultResetHandler` | `result_reset_handler.cpp` | 新工件复位：`executeResultResetTask`（清缓存/路径进度/检测标记，路径回首条；递增 `workpieceGeneration` 作废在途回调） |

---

## 6. 新增 Handler 步骤

1. 在 `handlers/` 增加 `xxx_handler.h/.cpp`，实现 `ITaskHandler`
2. 在 `task_handler_registry.cpp` 注册
3. 在 `execute(TaskHandlerContext& ctx)` 中编写业务，**只通过 `ctx.host` 与 `ctx.activeTask` 交互**
4. 若需新的 PLC/状态能力，在 `PlcTaskHost` 增加虚方法并在 `StateMachine` 实现 — **不要** 使用 `friend` 或访问 `StateMachine` private 成员

示例：

```cpp
void MyHandler::execute(TaskHandlerContext& ctx)
{
    // 业务逻辑 …
    ctx.host.completeActiveTask(1, protocol::AckState::Completed, true);
    ctx.host.notifyXxxFinished(...);
}
```

---

## 7. PlcTaskHost 能力分组（摘要）

| 分组 | 代表方法 |
|------|----------|
| 连接/服务 | `isModbusConnected()`, `modbusService()`, `mechEyeService()`, `visionPipelineService()` |
| 任务收尾 | `completeActiveTask()`, `setTaskProgress()`, `publishIpcStatus()` |
| 寄存器 | `writeLoadGraspResult()`, `writeFloatPlaceholder()`, `clearScanSegmentDoneRegisters()` … |
| 扫描 | `completeScanSegmentCapture()`, `notifyScanStarted()`, `resetScanSegmentCache()` |
| 检测 | `evaluateInspectionForActiveTask()`, `finishInspection()` |
| HMI 事件 | `notifyLoadGraspFinished()`, `notifyScanStarted()` 等（内部 `emit` 信号） |

完整列表见 [`plc_task_host.h`](../../modules/flow_control/include/scan_tracking/flow_control/plc_task_host.h)。

---

## 8. 与第一工位差异（勿回迁）

以下 API **仅存在于第一工位**，本仓已删除，勿从 `scan-tracking` 抄回：

- 多路径点云缓存、`loadMergedPointCloudForInspection`
- LBN 标定 / `applySegmentPoseStitching` / 后台 PCL refinement
- `executeBypassActiveTask`、`TrackingService`
- `StateMachine::executeXxxTask()` 成员函数模式（已改为 Handler 内联业务）

---

## 9. 关联文档

| 文档 | 用途 |
|------|------|
| [扫描路径配置说明](./扫描路径配置说明.md) | ScanSegment 与 JSON |

---

*最后更新：2026-06-16*
