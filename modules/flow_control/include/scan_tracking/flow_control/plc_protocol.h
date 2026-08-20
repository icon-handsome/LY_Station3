/**
 * @file plc_protocol.h
 * @brief PLC与IPC之间的Modbus通讯协议定义
 * 
 * 本文件定义了扫描跟踪系统中PLC与IPC之间
 * 的Modbus TCP通讯协议。协议采用寄存器映射方式，通过读写特定的寄存器地址实现
 * 双向数据交换和状态同步。
 * 
 * 通讯架构：
 * - PLC作为Modbus主站，负责发起读写请求
 * - IPC作为Modbus从站，响应PLC的请求并更新寄存器数据
 */

#pragma once

#include <cmath>
#include <cstring>

#include <QtCore/QByteArray>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

namespace scan_tracking::flow_control::protocol {

/**
 * @enum Stage
 * @brief 工作流程阶段枚举
 * 
 * 定义扫描跟踪系统的完整工作流程阶段，每个阶段对应一个具体的工艺步骤。
 * 阶段按顺序执行，形成完整的扫描检测流程。
 */
enum class Stage : quint16 {
    Idle = 0,                 ///< 空闲状态：系统待机，无任务执行
    SafetyMonitoring = 1,     ///< 安全监控：实时监测安全设备状态（光栅、急停等）
    SelfCheck = 2,            ///< 自检阶段：检查各子系统（相机、传感器等）是否正常
    LoadMaterialCheck = 3,    ///< 上料检测：判断上料工位是否有物料需要抓取
    LoadGrasp = 4,            ///< 上料抓取：计算抓取点坐标并执行抓取动作
    StationMaterialCheck = 5, ///< 工位检材：确认扫描工位是否已放置物料
    PoseCheck = 6,            ///< 位姿校验：验证物料通孔位姿是否符合要求
    ScanSegment = 7,          ///< 分段扫描：扫描机械臂外部多点位采集
    Inspection = 8,           ///< 综合检测：进行OCR识别、缺陷检测和尺寸测量
    UnloadCalc = 9,           ///< 卸料计算：计算下料区卸料点坐标
    ResultReset = 10,         ///< 结果复位：清空检测结果，准备下一轮循环
    TelescopicScan = 11,      ///< 伸缩杆内部扫描：纵缝/环缝步进采集（与 ScanSegment 触发分离）
};

/**
 * @enum IpcState
 * @brief IPC系统状态枚举
 * 
 * 反映IPC当前的运行状态，PLC根据此状态判断是否可以发送新指令。
 */
enum class IpcState : quint16 {
    Uninitialized = 0,  ///< 未初始化：IPC刚启动，尚未完成初始化
    Initializing = 1,   ///< 初始化中：正在加载配置、连接设备等
    Ready = 2,          ///< 就绪状态：可以接收并执行PLC指令
    Busy = 3,           ///< 忙碌状态：正在执行任务，暂不接收新指令
    Paused = 4,         ///< 暂停状态：任务被临时挂起
    Fault = 5,          ///< 故障状态：发生错误，需要人工干预
};

/**
 * @enum AckState
 * @brief 应答状态枚举
 * 
 * 用于触发-应答机制中的状态反馈，表示IPC对PLC触发信号的处理进度。
 */
enum class AckState : quint16 {
    Idle = 0,       ///< 空闲：未收到触发或已完成复位
    Running = 1,    ///< 执行中：正在处理触发的任务（段扫已跳过，仅终态 Ack=2/3）
    Completed = 2,  ///< 已完成：任务成功执行完毕（段扫=全部参与相机拍完）
    Failed = 3,     ///< 失败：任务执行出错
};

/**
 * @struct TriggerDefinition
 * @brief 触发器定义结构
 * 
 * 定义一个完整的触发-应答-结果三元组，用于PLC与IPC之间的任务协调。
 * 每个触发器对应一个具体的工艺动作，包含触发信号、应答信号和结果数据的寄存器偏移量。
 */
struct TriggerDefinition {
    const char* name = "";              ///< 触发器名称标识
    int trigOffset = 0;                 ///< 触发信号缓冲下标（40024 → 24）
    int ackOffset = 0;                  ///< 应答信号寄存器偏移量（IPC写入，PLC读取）
    int resOffset = 0;                  ///< 结果代码寄存器偏移量（IPC写入，PLC读取）
    Stage stage = Stage::Idle;          ///< 对应的工艺流程阶段
    int defaultTimeoutSeconds = 0;      ///< 默认超时时间（秒），超过此时间未完成则判定为故障
};

/**
 * @namespace registers
 * @brief Modbus寄存器地址定义
 * 
 * 定义PLC与IPC通讯时使用的所有Modbus保持寄存器的地址映射。
 * 寄存器分为两个主要区域：
 * - 命令区（0-39）：PLC向IPC发送控制指令和参数
 * - 结果区（100-183）：IPC向PLC返回执行结果和状态信息
 * 
 * 注意：本现场 Qt Modbus 映射为 modbusIndex = PLC地址 - 40000（40015→下标15）。
 * 与地址表一致：完整地址 = 40000 + modbusIndex，plcOffset = modbusIndex。
 */
namespace registers {

constexpr int kPlcAddressOrigin = 40000; ///< PLC 保持寄存器地址原点（40015→下标15）

/// 缓冲下标 → 完整 PLC 地址（15 → 40015）
inline constexpr int holdingRegisterAddress(int modbusIndex)
{
    return kPlcAddressOrigin + modbusIndex;
}

/// 缓冲下标 → 地址表偏移列（与 40015 的“15”一致）
inline constexpr int plcTableOffset(int modbusIndex)
{
    return modbusIndex;
}

/// 完整 PLC 地址 → 缓冲下标（40017 → 17）
inline constexpr int modbusIndexFromPlcAddress(int plcAddress)
{
    return plcAddress - kPlcAddressOrigin;
}

/// PLC 下发 REAL 时未做 CDAB：按 [高字, 低字] 拼成 IEEE754
inline float realFromPlcWords(quint16 word0, quint16 word1)
{
    const quint32 raw = (static_cast<quint32>(word0) << 16) | static_cast<quint32>(word1);
    float value = 0.0f;
    static_assert(sizeof(value) == sizeof(raw));
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

/// PLC 下发 REAL 时未做 CDAB：按 CDAB（低字在前）拼成 IEEE754
inline float realFromCdabWords(quint16 word0, quint16 word1)
{
    const quint32 raw = (static_cast<quint32>(word1) << 16) | static_cast<quint32>(word0);
    float value = 0.0f;
    static_assert(sizeof(value) == sizeof(raw));
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

/**
 * @brief 将 PLC 原始寄存器值解析为无符号整数
 *
 * PLC 常把 REAL（如 1.0f → 0x3F80 高字）直接写入单寄存器而未做字序转换。
 * 依次尝试高字在前、CDAB 等组合，优先取可解释为整数的合理解。
 */
inline quint16 plcAnalogToUInt16(quint16 word0, quint16 word1 = 0)
{
    if (word0 == 0 && word1 == 0) {
        return 0;
    }
    if (word1 == 0 && word0 > 0 && word0 <= 1000) {
        return word0;
    }

    const float candidates[] = {
        realFromPlcWords(word0, word1),
        realFromPlcWords(word1, word0),
        realFromCdabWords(word0, word1),
        realFromCdabWords(word1, word0),
    };

    for (float candidate : candidates) {
        if (!std::isfinite(candidate) || candidate < 0.0f || candidate > 65535.0f) {
            continue;
        }
        const float rounded = std::round(candidate);
        if (std::abs(candidate - rounded) < 0.001f) {
            return static_cast<quint16>(rounded);
        }
    }

    if (word0 > 0 && word0 <= 1000) {
        return word0;
    }
    return static_cast<quint16>(std::round(realFromPlcWords(word0, word1)));
}

// ==================== 寄存器区块定义 ====================

constexpr int kCommandBlockStart = 0;    ///< 命令区起始地址：PLC→IPC的控制指令区域（0 基偏移）
constexpr int kCommandBlockSize = 51;    ///< 命令区大小：40001~40051（modbusIndex 0~50）
constexpr int kResultBlockStart = 101;   ///< 结果区起始：40101（modbusIndex=101）
constexpr int kResultBlockSize = 85;     ///< 结果区大小：40101~40185（含 ScanPathIdEcho）

// ==================== 命令区寄存器（PLC → IPC）====================

constexpr int kPlcHeartbeat = modbusIndexFromPlcAddress(40001);
constexpr int kPlcSystemState = modbusIndexFromPlcAddress(40002);
constexpr int kStationWorkMode = modbusIndexFromPlcAddress(40003);
constexpr int kFlowEnable = modbusIndexFromPlcAddress(40004);
constexpr int kSafetyStatusWord = modbusIndexFromPlcAddress(40005);
constexpr int kTaskIdHigh = modbusIndexFromPlcAddress(40011);
constexpr int kTaskIdLow = modbusIndexFromPlcAddress(40012);
constexpr int kProductType = modbusIndexFromPlcAddress(40013);
constexpr int kRecipeId = modbusIndexFromPlcAddress(40014);
constexpr int kArmScanSegmentIndex = modbusIndexFromPlcAddress(40015);        ///< AO47：机械臂相机段号（单字）
constexpr int kTelescopicScanSegmentIndex = modbusIndexFromPlcAddress(40016); ///< AO48：伸缩杆相机段号（单字）
constexpr int kRequestTimeoutSeconds = modbusIndexFromPlcAddress(40017);
constexpr int kRobotStatusWord = modbusIndexFromPlcAddress(40018);        ///< 机械臂状态字（PLC 转发埃斯顿 Robot 40004）

/// 埃斯顿 ER 系列 Robot Modbus 40004 状态字位定义（RCS2 V2.0，PLC 原样转发至 IPC 40018）
namespace robot_status_bits {
constexpr quint16 kManualMode = 1u << 0;       ///< 手动操作模式
constexpr quint16 kAutoMode = 1u << 1;         ///< 自动操作模式
constexpr quint16 kRemoteMode = 1u << 2;       ///< 远程操作模式
constexpr quint16 kEnabled = 1u << 3;          ///< 使能（上励磁）
constexpr quint16 kSystemRunning = 1u << 4;    ///< 系统运行
constexpr quint16 kSystemError = 1u << 5;      ///< 系统错误
constexpr quint16 kProgramRunning = 1u << 6;   ///< 程序运行
constexpr quint16 kRobotMoving = 1u << 7;      ///< 机器人运动
constexpr quint16 kSystemAlarm = 1u << 10;     ///< 系统报警
constexpr quint16 kServoAlarm = 1u << 11;     ///< 伺服报警
constexpr quint16 kEmergencyStop = 1u << 12;   ///< 急停
constexpr quint16 kSafetyDoor = 1u << 13;      ///< 安全门
}  // namespace robot_status_bits

// --- 机械臂末端中心点位姿（PLC → IPC，CDAB FLOAT32，单位 mm / deg）---
constexpr int kRobotTcpX = modbusIndexFromPlcAddress(40029);
constexpr int kRobotTcpY = modbusIndexFromPlcAddress(40031);
constexpr int kRobotTcpZ = modbusIndexFromPlcAddress(40033);
constexpr int kRobotTcpRx = modbusIndexFromPlcAddress(40035);
constexpr int kRobotTcpRy = modbusIndexFromPlcAddress(40037);
constexpr int kRobotTcpRz = modbusIndexFromPlcAddress(40039);

// --- 辅机状态（PLC → IPC，显控监视）---
constexpr int kTelescopicRodStatus = modbusIndexFromPlcAddress(40041);
constexpr int kRollerSetFreqHz = modbusIndexFromPlcAddress(40042);
constexpr int kRollerRunFreqHz = modbusIndexFromPlcAddress(40043);
constexpr int kElectromagnetStatus = modbusIndexFromPlcAddress(40044);
constexpr int kEstopButtonStatus = modbusIndexFromPlcAddress(40045);

// --- 命令区扩展（v0.3+：伸缩杆触发；v0.3.1：扫描路径号）---
constexpr int kTrigTelescopicScan = modbusIndexFromPlcAddress(40046);
/// 当前扫描路径号（PLC→IPC，**PLC 必写**）。对应 scan_paths.json 的 pathId，从 1 开始；
/// 0=未指定（IPC 沿用当前活跃路径）。须在相关 Trig 置 1 前写好。
constexpr int kScanPathId = modbusIndexFromPlcAddress(40047);

/// 六轴位姿（x/y/z + rx/ry/rz）
struct Pose6f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
    float rz = 0.0f;
};

/// 从命令块读取单个 CDAB FLOAT32（与 Load_X / Unload_X 字序一致）
inline float readFloatFromCommandBlock(const QVector<quint16>& commandBlock, int startOffset)
{
    return realFromCdabWords(commandBlock.value(startOffset), commandBlock.value(startOffset + 1));
}

/// 从命令块读取机械臂末端中心点位姿（40029~40040）
inline Pose6f readRobotTcpPoseFromCommandBlock(const QVector<quint16>& commandBlock)
{
    return {
        readFloatFromCommandBlock(commandBlock, kRobotTcpX),
        readFloatFromCommandBlock(commandBlock, kRobotTcpY),
        readFloatFromCommandBlock(commandBlock, kRobotTcpZ),
        readFloatFromCommandBlock(commandBlock, kRobotTcpRx),
        readFloatFromCommandBlock(commandBlock, kRobotTcpRy),
        readFloatFromCommandBlock(commandBlock, kRobotTcpRz),
    };
}

/// 按触发阶段从对应单字寄存器解析段号：
/// - ScanSegment → 40015（AO47，机械臂）
/// - TelescopicScan → 40016（AO48，伸缩杆）
/// 其他阶段默认读机械臂段号（仅用于监视/兜底）。
inline quint16 resolveScanSegmentIndexFromBlock(const QVector<quint16>& commandBlock, Stage stage)
{
    const int modbusIndex = (stage == Stage::TelescopicScan)
        ? kTelescopicScanSegmentIndex
        : kArmScanSegmentIndex;
    return plcAnalogToUInt16(commandBlock.value(modbusIndex), 0);
}

// ==================== 结果区寄存器（IPC → PLC）====================
constexpr int kIpcHeartbeat = modbusIndexFromPlcAddress(40101);
constexpr int kIpcSystemState = modbusIndexFromPlcAddress(40102);
constexpr int kIpcCurrentStage = modbusIndexFromPlcAddress(40103);
constexpr int kIpcAlarmLevel = modbusIndexFromPlcAddress(40104);
constexpr int kIpcAlarmCode = modbusIndexFromPlcAddress(40105);
constexpr int kIpcWarnCode = modbusIndexFromPlcAddress(40106);
constexpr int kIpcReady = modbusIndexFromPlcAddress(40107);
constexpr int kIpcDataValid = modbusIndexFromPlcAddress(40108);
constexpr int kIpcProgress = modbusIndexFromPlcAddress(40109);

// --- 设备状态监控 ---
constexpr int kDeviceOnlineWord0 = modbusIndexFromPlcAddress(40110);
constexpr int kDeviceOnlineWord1 = modbusIndexFromPlcAddress(40111);
constexpr int kDeviceFaultWord0 = modbusIndexFromPlcAddress(40112);
constexpr int kDeviceFaultWord1 = modbusIndexFromPlcAddress(40113);

// --- 任务ID回显 ---
constexpr int kTaskIdEchoHigh = modbusIndexFromPlcAddress(40114);
constexpr int kTaskIdEchoLow = modbusIndexFromPlcAddress(40115);

// ==================== 触发器应答与结果寄存器 ====================

// --- 上料抓取（LoadGrasp）---
constexpr int kAckLoadGrasp = modbusIndexFromPlcAddress(40116);
constexpr int kResLoadGrasp = modbusIndexFromPlcAddress(40117);
constexpr int kLoadX = modbusIndexFromPlcAddress(40118);
constexpr int kLoadY = modbusIndexFromPlcAddress(40120);
constexpr int kLoadZ = modbusIndexFromPlcAddress(40122);
constexpr int kLoadRx = modbusIndexFromPlcAddress(40124);
constexpr int kLoadRy = modbusIndexFromPlcAddress(40126);
constexpr int kLoadRz = modbusIndexFromPlcAddress(40128);

// --- 工位检材（StationMaterialCheck）---
constexpr int kAckStationMaterialCheck = modbusIndexFromPlcAddress(40130);
constexpr int kResStationMaterialCheck = modbusIndexFromPlcAddress(40131);

// --- 位姿校验（PoseCheck）---
constexpr int kAckPoseCheck = modbusIndexFromPlcAddress(40132);
constexpr int kResPoseCheck = modbusIndexFromPlcAddress(40133);
constexpr int kPoseDeviationMm = modbusIndexFromPlcAddress(40134);

// --- 分段扫描（ScanSegment）---
constexpr int kAckScanSegment = modbusIndexFromPlcAddress(40135);
constexpr int kResScanSegment = modbusIndexFromPlcAddress(40136);
constexpr int kScanSegmentDoneIndex = modbusIndexFromPlcAddress(40137);
constexpr int kScanImageCount = modbusIndexFromPlcAddress(40138);
constexpr int kScanCloudFrameCount = modbusIndexFromPlcAddress(40139);

// --- 伸缩杆内部扫描（TelescopicScan，v0.3+ 结果区 40180~40184）---
constexpr int kAckTelescopicScan = modbusIndexFromPlcAddress(40180);
constexpr int kResTelescopicScan = modbusIndexFromPlcAddress(40181);
constexpr int kTelescopicScanDoneIndex = modbusIndexFromPlcAddress(40182);
constexpr int kTelescopicImageCount = modbusIndexFromPlcAddress(40183);
constexpr int kTelescopicCloudFrameCount = modbusIndexFromPlcAddress(40184);

/// IPC 当前生效扫描路径号回显（可选；供 HMI/调试，**PLC 无需读取**）。
constexpr int kScanPathIdEcho = modbusIndexFromPlcAddress(40185);

// --- 综合检测（Inspection）---
constexpr int kAckInspection = modbusIndexFromPlcAddress(40140);
constexpr int kResInspection = modbusIndexFromPlcAddress(40141);
constexpr int kNgReasonWord0 = modbusIndexFromPlcAddress(40142);
constexpr int kNgReasonWord1 = modbusIndexFromPlcAddress(40143);
constexpr int kMeasureItemCount = modbusIndexFromPlcAddress(40144);

// --- 卸料计算（UnloadCalc）---
constexpr int kAckUnloadCalc = modbusIndexFromPlcAddress(40145);
constexpr int kResUnloadCalc = modbusIndexFromPlcAddress(40146);
constexpr int kUnloadX = modbusIndexFromPlcAddress(40147);
constexpr int kUnloadY = modbusIndexFromPlcAddress(40149);
constexpr int kUnloadZ = modbusIndexFromPlcAddress(40151);
constexpr int kUnloadRx = modbusIndexFromPlcAddress(40153);
constexpr int kUnloadRy = modbusIndexFromPlcAddress(40155);
constexpr int kUnloadRz = modbusIndexFromPlcAddress(40157);

// --- 自检（SelfCheck）---
constexpr int kAckSelfCheck = modbusIndexFromPlcAddress(40159);
constexpr int kResSelfCheck = modbusIndexFromPlcAddress(40160);
constexpr int kSelfCheckFailWord0 = modbusIndexFromPlcAddress(40161);
constexpr int kSelfCheckFailWord1 = modbusIndexFromPlcAddress(40162);

// --- 条码读取（CodeRead）---
constexpr int kAckCodeRead = modbusIndexFromPlcAddress(40163);
constexpr int kResCodeRead = modbusIndexFromPlcAddress(40164);
constexpr int kCodeValueAscii = modbusIndexFromPlcAddress(40165);
constexpr int kCodeValueRegisterCount = 8;

// --- 安全与复位 ---
constexpr int kIpcSafetyActionWord = modbusIndexFromPlcAddress(40173);
constexpr int kAckResultReset = modbusIndexFromPlcAddress(40174);
constexpr int kResResultReset = modbusIndexFromPlcAddress(40175);

}  // namespace registers

/// IPC_SafetyAction_Word (40173) 位域
namespace safety_bits {
constexpr quint16 kAiPersonIntrusion = 1u << 0;  ///< Bit0: AI 监控报警（人员越界）
}  // namespace safety_bits

/**
 * @brief 获取所有触发器定义的引用
 * 
 * 返回系统中定义的所有触发器的静态数组，包含触发信号、应答信号、结果信号的
 * 寄存器偏移量以及对应的工艺阶段和超时时间。
 * 
 * @return const QVector<TriggerDefinition>& 触发器定义列表
 */
inline const QVector<TriggerDefinition>& triggerDefinitions()
{
    static const QVector<TriggerDefinition> definitions = {
        {"Trig_TelescopicScan", registers::kTrigTelescopicScan, registers::kAckTelescopicScan, registers::kResTelescopicScan, Stage::TelescopicScan, 600},
        {"Trig_LoadGrasp", registers::modbusIndexFromPlcAddress(40020), registers::kAckLoadGrasp, registers::kResLoadGrasp, Stage::LoadGrasp, 10},
        {"Trig_StationMaterialCheck", registers::modbusIndexFromPlcAddress(40021), registers::kAckStationMaterialCheck, registers::kResStationMaterialCheck, Stage::StationMaterialCheck, 5},
        {"Trig_PoseCheck", registers::modbusIndexFromPlcAddress(40022), registers::kAckPoseCheck, registers::kResPoseCheck, Stage::PoseCheck, 5},
        {"Trig_ScanSegment", registers::modbusIndexFromPlcAddress(40023), registers::kAckScanSegment, registers::kResScanSegment, Stage::ScanSegment, 600},
        {"Trig_Inspection", registers::modbusIndexFromPlcAddress(40024), registers::kAckInspection, registers::kResInspection, Stage::Inspection, 60},
        {"Trig_UnloadCalc", registers::modbusIndexFromPlcAddress(40025), registers::kAckUnloadCalc, registers::kResUnloadCalc, Stage::UnloadCalc, 10},
        {"Trig_SelfCheck", registers::modbusIndexFromPlcAddress(40026), registers::kAckSelfCheck, registers::kResSelfCheck, Stage::SelfCheck, 120},
        {"Trig_CodeRead", registers::modbusIndexFromPlcAddress(40027), registers::kAckCodeRead, registers::kResCodeRead, Stage::Inspection, 10},
        {"Trig_ResultReset", registers::modbusIndexFromPlcAddress(40028), registers::kAckResultReset, registers::kResResultReset, Stage::ResultReset, 10},
    };
    return definitions;
}

/**
 * @brief 根据触发信号偏移量查找触发器定义
 * 
 * 遍历所有触发器定义，找到与给定触发信号寄存器偏移量匹配的触发器。
 * 
 * @param trigOffset 触发信号寄存器偏移量
 * @return const TriggerDefinition* 找到的触发器定义指针，未找到返回nullptr
 */
inline const TriggerDefinition* triggerByOffset(int trigOffset)
{
    const auto& definitions = triggerDefinitions();
    for (const auto& definition : definitions) {
        if (definition.trigOffset == trigOffset) {
            return &definition;
        }
    }
    return nullptr;
}

/**
 * @brief 获取触发器的默认结果码
 * 
 * 对于给定的触发器定义，返回其成功执行时的默认结果代码。
 * 目前统一返回1表示成功，可根据实际需求扩展。
 * 
 * @param definition 触发器定义引用
 * @return quint16 默认结果码
 */
inline quint16 defaultResCodeFor(const TriggerDefinition& definition)
{
    Q_UNUSED(definition);
    return 1;
}

/**
 * @brief 获取触发器的名称字符串
 * 
 * 将触发器定义中的C风格字符串转换为Qt字符串，便于日志输出和界面显示。
 * 
 * @param definition 触发器定义引用
 * @return QString 触发器名称
 */
inline QString triggerName(const TriggerDefinition& definition)
{
    return QString::fromLatin1(definition.name);
}

}  // namespace scan_tracking::flow_control::protocol
