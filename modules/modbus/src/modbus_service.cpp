/**
 * @brief Modbus TCP Server 实现（IPC 作为从站）
 *
 * PLC 作为 Master 主动来读写 IPC 的寄存器映射区。
 * IPC 只需维护好寄存器区域，PLC 来写命令区时触发信号通知状态机。
 */

#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/common/config_manager.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QMutexLocker>
#include <QtSerialBus/QModbusDataUnit>
#include <QtSerialBus/QModbusDevice>

namespace scan_tracking::modbus {

Q_LOGGING_CATEGORY(LOG_MODBUS, "modbus")

namespace {
/// 寄存器总大小：覆盖 40001-40200（offset 0-199），足够容纳命令区+结果区+预留
constexpr int kTotalRegisterCount = 200;
/// 命令区（PLC 写入）：offset 0-50，覆盖 PLC 40001~40051
constexpr int kCommandBlockStart = 0;
constexpr int kCommandBlockSize = 51;
/// 结果区（IPC 写入，PLC 读取）：offset 101-185（40101~40185，含 ScanPathIdEcho）
constexpr int kResultBlockStart = 101;  // 40101，与 plc_protocol 一致（40000+下标）
constexpr int kResultBlockSize = 85;
}

ModbusService::ModbusService(QObject* parent)
    : QObject(parent)
    , m_server(new QModbusTcpServer(this))
{
    connect(m_server, &QModbusTcpServer::stateChanged, this, &ModbusService::onStateChanged);
    connect(m_server, &QModbusTcpServer::errorOccurred, this, &ModbusService::onModbusError);
    connect(m_server, &QModbusTcpServer::dataWritten, this, &ModbusService::onDataWritten);
}

ModbusService::~ModbusService()
{
    disconnectDevice();
}

void ModbusService::initRegisterMap()
{
    // 初始化 Holding Registers 区域（所有值为 0）
    QModbusDataUnitMap reg;
    reg.insert(QModbusDataUnit::HoldingRegisters,
               {QModbusDataUnit::HoldingRegisters, 0, kTotalRegisterCount});
    m_server->setMap(reg);

    qInfo(LOG_MODBUS) << "寄存器映射已初始化，总大小:" << kTotalRegisterCount
                      << "命令区: 0-45, 结果区: 100-183";
}

bool ModbusService::connectDevice()
{
    if (m_server == nullptr) {
        return false;
    }

    if (m_server->state() == QModbusDevice::ConnectedState) {
        qInfo(LOG_MODBUS) << "Server 已在监听中，忽略重复启动。";
        return true;
    }

    auto* configManager = common::ConfigManager::instance();
    if (configManager == nullptr) {
        qCritical(LOG_MODBUS) << "ConfigManager 未加载";
        emit errorOccurred(QStringLiteral("ConfigManager 未加载"));
        return false;
    }

    const auto& conf = configManager->modbusConfig();
    m_unitId = conf.unitId;
    m_port = static_cast<quint16>(conf.port);

    // 初始化寄存器映射
    initRegisterMap();

    // 设置 Server 参数
    m_server->setConnectionParameter(QModbusDevice::NetworkPortParameter, m_port);
    m_server->setConnectionParameter(QModbusDevice::NetworkAddressParameter, conf.host);
    m_server->setServerAddress(m_unitId);

    qInfo(LOG_MODBUS).noquote()
        << QStringLiteral("正在启动 Modbus TCP Server，监听:")
        << conf.host << QStringLiteral(":") << m_port
        << QStringLiteral(" unitId=") << m_unitId;

    if (!m_server->connectDevice()) {
        const QString error = m_server->errorString();
        qCritical(LOG_MODBUS).noquote() << "Modbus Server 启动失败：" << error;
        emit errorOccurred(error);
        return false;
    }

    return true;
}

void ModbusService::disconnectDevice()
{
    if (m_server == nullptr) {
        return;
    }

    if (m_server->state() != QModbusDevice::UnconnectedState) {
        qInfo(LOG_MODBUS) << "正在停止 Modbus Server。";
        m_server->disconnectDevice();
    }
    m_listening = false;
}

bool ModbusService::isConnected() const
{
    return m_server != nullptr && m_server->state() == QModbusDevice::ConnectedState;
}

bool ModbusService::readRegisters(int startAddress, quint16 numberOfEntries)
{
    QMutexLocker locker(&m_mutex);
    if (!isConnected()) {
        emit registerReadFailed(startAddress, QStringLiteral("Server 未在监听"));
        return false;
    }

    QModbusDataUnit unit(QModbusDataUnit::HoldingRegisters, startAddress, numberOfEntries);
    if (!m_server->data(&unit)) {
        emit registerReadFailed(startAddress, QStringLiteral("读取本地寄存器失败"));
        return false;
    }

    QVector<quint16> values;
    values.reserve(static_cast<int>(unit.valueCount()));
    for (uint i = 0; i < unit.valueCount(); ++i) {
        values.push_back(unit.value(i));
    }

    emit registersRead(startAddress, values);
    return true;
}

bool ModbusService::writeRegister(int startAddress, quint16 value)
{
    return writeRegisters(startAddress, {value});
}

bool ModbusService::writeRegisters(int startAddress, const QVector<quint16>& values)
{
    QMutexLocker locker(&m_mutex);
    if (!isConnected()) {
        qWarning(LOG_MODBUS) << "Server 未在监听，跳过写入。";
        return false;
    }

    QModbusDataUnit unit(QModbusDataUnit::HoldingRegisters, startAddress, static_cast<quint16>(values.size()));
    for (int i = 0; i < values.size(); ++i) {
        unit.setValue(i, values[i]);
    }

    if (!m_server->setData(unit)) {
        const QString error = QStringLiteral("写入本地寄存器失败 offset=%1 count=%2")
                                  .arg(startAddress).arg(values.size());
        qWarning(LOG_MODBUS).noquote() << error;
        emit registerWriteFailed(startAddress, error);
        return false;
    }

    return true;
}

bool ModbusService::resetIpcResultBlock()
{
    qInfo(LOG_MODBUS).noquote()
        << QStringLiteral("正在清零 IPC 结果区寄存器 40101-40184（共 ")
        << kResultBlockSize << QStringLiteral(" 个）");
    return writeRegisters(kResultBlockStart, QVector<quint16>(kResultBlockSize, 0));
}

void ModbusService::onDataWritten(QModbusDataUnit::RegisterType table, int address, int size)
{
    // 只关心 Holding Registers 的写入
    if (table != QModbusDataUnit::HoldingRegisters) {
        return;
    }

    // 只关心命令区（offset 0-50）的写入——这是 PLC 发来的命令
    if (address >= kCommandBlockStart && address < kCommandBlockStart + kCommandBlockSize) {
        // 读取完整命令区
        QModbusDataUnit unit(QModbusDataUnit::HoldingRegisters, kCommandBlockStart, kCommandBlockSize);
        if (!m_server->data(&unit)) {
            qWarning(LOG_MODBUS) << "读取命令区失败";
            return;
        }

        QVector<quint16> values;
        values.reserve(kCommandBlockSize);
        for (int i = 0; i < kCommandBlockSize; ++i) {
            values.push_back(unit.value(i));
        }

        // 发射信号，与原来的 registersRead 完全兼容
        emit registersRead(kCommandBlockStart, values);

        ++m_writeByPlcLogCounter;
        if (m_writeByPlcLogCounter <= 3 || (m_writeByPlcLogCounter % 100) == 0) {
            qDebug(LOG_MODBUS).noquote()
                << QStringLiteral("PLC 写入命令区 | address=") << address
                << QStringLiteral(" size=") << size
                << QStringLiteral(" totalWrites=") << m_writeByPlcLogCounter;
        }
    }
}

void ModbusService::onStateChanged(QModbusDevice::State state)
{
    if (state == QModbusDevice::ConnectedState) {
        m_listening = true;
        qInfo(LOG_MODBUS) << "Modbus TCP Server 已启动，正在监听。";
        emit connected();
    } else if (state == QModbusDevice::UnconnectedState) {
        if (m_listening) {
            m_listening = false;
            qWarning(LOG_MODBUS) << "Modbus Server 已停止。";
            emit disconnected();
        }
    }
}

void ModbusService::onModbusError(QModbusDevice::Error error)
{
    if (error == QModbusDevice::NoError) {
        return;
    }
    const QString errorString = m_server ? m_server->errorString() : QStringLiteral("未知错误");
    qWarning(LOG_MODBUS).noquote() << "Modbus Server 错误：" << errorString << "(代码" << error << ")";
    emit errorOccurred(errorString);
}

}  // namespace scan_tracking::modbus
