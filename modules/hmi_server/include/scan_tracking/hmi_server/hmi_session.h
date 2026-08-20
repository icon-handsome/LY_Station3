#pragma once

/**
 * @file hmi_session.h
 * @brief HMI 单客户端 TCP 会话管理
 *
 * 负责管理单个 Qt 显控客户端的 TCP 连接，包括：
 * - 基于 [4字节大端长度头 + JSON] 的粘包/半包解帧
 * - 心跳收发与超时检测
 * - 接收到的 JSON 消息转发给上层处理
 */

#include <QtCore/QObject>
#include <QtCore/QByteArray>
#include <QtCore/QTimer>
#include <QtCore/QJsonObject>

QT_BEGIN_NAMESPACE
class QTcpSocket;
QT_END_NAMESPACE

namespace scan_tracking {
namespace hmi_server {

class HmiSession : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param socket 已建立连接的 TCP 套接字（所有权转移到 HmiSession）
     * @param parent 父对象
     */
    explicit HmiSession(QTcpSocket* socket, QObject* parent = nullptr);
    ~HmiSession() override;

    /// 检查会话是否有效（套接字仍然连接）
    bool isConnected() const;

    /// 获取客户端地址描述（用于日志）
    QString peerDescription() const;

    /**
     * @brief 发送一条 JSON 消息给客户端
     * @param envelope 完整的协议 JSON 对象（包含 version/msgId/type/timestamp/payload）
     */
    void sendMessage(const QJsonObject& envelope);

    /// 主动断开连接
    void disconnect();

signals:
    /// 收到一条完整的 JSON 消息时发出
    void messageReceived(const QJsonObject& message);

    /// 连接断开时发出
    void disconnected();

    /// 心跳超时（超过约定时间未收到客户端心跳）时发出
    void heartbeatTimeout();

public slots:
    /// 重置心跳超时计时器（收到任何客户端消息时调用）
    void resetHeartbeatTimer();

private slots:
    /// 处理 socket 可读事件，解帧并分发消息
    void onReadyRead();

    /// 处理 socket 断开事件
    void onSocketDisconnected();

    /// 心跳超时回调
    void onHeartbeatTimeout();

private:
    QTcpSocket* m_socket = nullptr;         ///< TCP 套接字
    QByteArray m_buffer;                    ///< 接收缓冲区（用于粘包/半包处理）
    QTimer* m_heartbeatTimer = nullptr;     ///< 心跳超时定时器
    static constexpr int kHeartbeatTimeoutMs = 6000;  ///< 心跳超时时间（毫秒）
};

}  // namespace hmi_server
}  // namespace scan_tracking
