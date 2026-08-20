# Scan Tracking - Third Station IPC

第三工位 IPC 骨架，保留 Mech-Eye、海康智能相机、CXP 双目、LB 位姿、Modbus、HMI、Handler、段缓存和采集落盘。

默认使用 `config/scan_paths/station3_placeholder.json`，后续按第三工位实际路径、点位、相机数量和算法标识扩展。

第二工位焊缝、厚度、内表面、长度容积等算法模块与 SDK 已移除。

## 构建与运行

```powershell
cmd /c tools\scan_tracking_dev.cmd configure-debug
cmd /c tools\scan_tracking_dev.cmd build-debug
cmd /c tools\scan_tracking_dev.cmd run-debug
```

CMake 预设：`win-msvc2019-qtcore-ninja-debug` / `release`

## 文档

| 主题 | 文档 |
|------|------|
| 文档总索引 | [`docs/README.md`](docs/README.md) |
| flow_control 架构 | [`docs/guides/flow_control架构说明.md`](docs/guides/flow_control架构说明.md) |
| 扫描路径 JSON | [`docs/guides/扫描路径配置说明.md`](docs/guides/扫描路径配置说明.md) |
| Modbus 协议 | [`docs/protocols/封头检测工位PLC-IPC Modbus通信协议_v0.1.md`](docs/protocols/封头检测工位PLC-IPC%20Modbus通信协议_v0.1.md) |
| HMI 交接 | [`docs/hmi/HMI开发交接说明.md`](docs/hmi/HMI开发交接说明.md) |
