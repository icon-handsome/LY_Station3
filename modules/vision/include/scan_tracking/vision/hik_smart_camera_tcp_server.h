#pragma once

// 海康智能相机 TCP 服务端
// 监听端口，接收智能相机连接，处理心跳和指令通讯

#include <QtCore/QObject>
#include <QtCore/QMap>
#include <QtCore/QTimer>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>

namespace scan_tracking {
namespace vision {

// 智能相机会话（单个相机连接）
class HikSmartCameraSession : public QObject {
    Q_OBJECT

public:
    explicit HikSmartCameraSession(QTcpSocket* socket, QObject* parent = nullptr);
    ~HikSmartCameraSession() override;

    QString cameraIp() const { return m_cameraIp; }
    quint16 cameraPort() const { return m_cameraPort; }
    bool isConnected() const;
    qint64 lastHeartbeatTime() const { return m_lastHeartbeatMs; }
    bool everReceivedHeartbeat() const { return m_receivedHeartbeat; }

    // 发送指令
    bool sendCommand(const QString& command);
    bool sendStartCapture();  // 发送 "start\r\n"

signals:
    void heartbeatReceived(QString cameraIp);
    void commandReceived(QString cameraIp, QString command);
    void imageDataReceived(QString cameraIp, QByteArray imageData);  // 预留：接收图像数据
    void disconnected(QString cameraIp);
    void error(QString cameraIp, QString errorMessage);

private slots:
    void onReadyRead();
    void onDisconnected();
    void onSocketError(QAbstractSocket::SocketError socketError);

private:
    void drainReceiveBuffer();
    void processReceivedData(const QByteArray& data);
    void updateHeartbeat();

    QTcpSocket* m_socket = nullptr;
    QString m_cameraIp;
    quint16 m_cameraPort = 0;
    QByteArray m_receiveBuffer;  // 接收缓冲区
    qint64 m_lastHeartbeatMs = 0;
    bool m_receivedHeartbeat = false;
};

// TCP 服务端
class HikSmartCameraTcpServer : public QObject {
    Q_OBJECT

public:
    explicit HikSmartCameraTcpServer(QObject* parent = nullptr);
    ~HikSmartCameraTcpServer() override;

    bool start(const QString& listenIp, quint16 port);
    void stop();

    bool isListening() const;
    QString serverAddress() const;
    quint16 serverPort() const;

    // 获取已连接的相机列表
    QStringList connectedCameras() const;

    // 向指定相机发送指令
    bool sendCommandToCamera(const QString& cameraIp, const QString& command);
    bool sendStartCaptureToCamera(const QString& cameraIp);

signals:
    void serverStarted(QString listenIp, quint16 port);
    void serverStopped();
    void cameraConnected(QString cameraIp, quint16 cameraPort);
    void cameraDisconnected(QString cameraIp);
    void heartbeatReceived(QString cameraIp);
    void commandReceived(QString cameraIp, QString command);
    void imageDataReceived(QString cameraIp, QByteArray imageData);
    void error(QString errorMessage);

private slots:
    void onNewConnection();
    void onSessionDisconnected(QString cameraIp);
    void onHeartbeatTimeout();

private:
    void startHeartbeatMonitor();
    void stopHeartbeatMonitor();

    QTcpServer* m_tcpServer = nullptr;
    QMap<QString, HikSmartCameraSession*> m_sessions;  // IP -> Session
    QTimer* m_heartbeatTimer = nullptr;
    int m_heartbeatTimeoutMs = 25000;  // 相机心跳间隔 10s，留足余量
};

}  // namespace vision
}  // namespace scan_tracking
