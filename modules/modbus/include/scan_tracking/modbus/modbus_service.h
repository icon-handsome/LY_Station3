#pragma once

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QString>
#include <QtSerialBus/QModbusTcpServer>
#include <QtCore/QMutex>

namespace scan_tracking {
namespace modbus {

/**
 * @brief Modbus TCP Server 模块（IPC 作为从站）
 *
 * 架构：
 * - IPC 作为 Modbus TCP Server（Slave），监听 502 端口
 * - PLC 作为 Modbus TCP Client（Master），主动读写 IPC 的寄存器
 * - PLC 每 100ms 用 FuncId=16 写入命令区（40001-40045）
 * - PLC 每 100ms 用 FuncId=3 读取结果区（40101-40184）
 *
 * 接口说明：
 * - writeRegister/writeRegisters：IPC 侧主动更新结果区寄存器，PLC 下次读取时自动拿到新值
 * - registersWrittenByPLC 信号：当 PLC 写入命令区时触发，替代原来的轮询+registersRead
 */
class ModbusService : public QObject {
    Q_OBJECT
public:
    explicit ModbusService(QObject* parent = nullptr);
    ~ModbusService() override;

    /// 启动 Modbus TCP Server（替代原 connectDevice）
    bool connectDevice();

    /// 停止服务
    void disconnectDevice();

    /// Server 是否在监听
    bool isConnected() const;

    /// 从本地寄存器映射中读取（同步，供状态机主动查询命令区使用）
    bool readRegisters(int startAddress, quint16 numberOfEntries);

    /// 写入本地寄存器映射（PLC 下次读取时获得新值）
    bool writeRegister(int startAddress, quint16 value);
    bool writeRegisters(int startAddress, const QVector<quint16>& values);

    /// 程序退出时将 IPC 结果区（40101-40184）全部清零
    bool resetIpcResultBlock();

signals:
    /// PLC 写入命令区后触发（等同于原 registersRead）
    void registersRead(int startAddress, QVector<quint16> values);

    /// 读取失败（Server 模式下基本不会触发，保留接口兼容）
    void registerReadFailed(int startAddress, const QString& errorString);

    /// 写入失败（Server 模式下基本不会触发，保留接口兼容）
    void registerWriteFailed(int startAddress, const QString& errorString);

    /// 错误信号
    void errorOccurred(const QString& errorString);

    /// PLC 客户端连入
    void connected();

    /// PLC 客户端断开
    void disconnected();

private slots:
    void onDataWritten(QModbusDataUnit::RegisterType table, int address, int size);
    void onStateChanged(QModbusDevice::State state);
    void onModbusError(QModbusDevice::Error error);

private:
    void initRegisterMap();

    QModbusTcpServer* m_server = nullptr;
    int m_unitId = 1;
    quint16 m_port = 502;
    bool m_listening = false;
    QMutex m_mutex;

    // 命令区快照（用于变化检测）
    QVector<quint16> m_lastCommandBlock;

    int m_writeByPlcLogCounter = 0;
};

} // namespace modbus
} // namespace scan_tracking
