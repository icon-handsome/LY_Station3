#pragma once

// 海康智能相机 C 控制器（视觉传感器）。
//
// 纯 TCP + FTP 模式：IPC 作 TCP 服务端接收相机心跳与回包，通过 FTP 目录监控落图；
// 不经过 MVS SDK 打开设备，可与 SCMVS 同时运行。
// 由 StateMachine / HmiTcpServer 触发表面缺陷、焊缝、编号识别等拍照任务。

#include <QtCore/QHash>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QVector>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/vision/hik_smart_camera_ftp_monitor.h"

namespace scan_tracking {
namespace vision {

class HikSmartCameraTcpServer;
class HikSmartCameraFtpMonitor;

/// 智能相机 C 控制器生命周期状态
enum class HikCameraCState {
    Idle = 0,          ///< 未 start
    Initializing = 1,  ///< 正在启动 TCP 服务与 FTP 监控
    Ready = 2,         ///< 可接受拍照请求
    Error = 3,         ///< 不可恢复错误
    Stopped = 4,       ///< 已 stop
};

class HikCameraCController : public QObject {
    Q_OBJECT

public:
    explicit HikCameraCController(QObject* parent = nullptr);
    ~HikCameraCController() override;

    /* 根据 VisionConfig 启动 TCP 监听与 FTP 目录监控 */
    void start(const scan_tracking::common::VisionConfig& config);

    /* 停止 TCP/FTP 并进入 Stopped */
    void stop();

    bool isStarted() const { return m_started; }
    HikCameraCState state() const { return m_state; }

    bool isTcpServerRunning() const;
    bool isCameraConnected(const QString& cameraIp) const;
    bool isCameraConnectedToTcp() const;

    bool isFtpMonitorRunning() const;
    QString ftpDirectory() const;
    QStringList configuredCameraIps() const;

    /* 向已连接的智能相机发送拍照指令（TCP start），类型决定后续 FTP 文件名解析 */
    bool requestCapture(CaptureType type, const QString& cameraIp);
    bool requestCapture(CaptureType type = CaptureType::SurfaceDefect);

    /// 最近一次有效 OCR 文本（不含 hello/heartbeat）。
    QString lastOcrText() const { return m_lastOcrText; }
    QString lastOcrCameraIp() const { return m_lastOcrCameraIp; }

    /* 启用周期性自动拍照（联调/冒烟，默认 10s 间隔） */
    void enableTestMode(bool enable, int intervalMs = 10000);

signals:
    void stateChanged(scan_tracking::vision::HikCameraCState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);
    void captureCompleted(CaptureType type, QString cameraIp, QByteArray imageData);
    void imageReceived(CaptureType type, QString cameraIp, QString filePath, qint64 fileSize);
    /// 智能相机 TCP 回包解析出的编号/OCR 文本（已去掉 X/PX 前缀与末尾分号）。
    void ocrTextReceived(QString cameraIp, QString text);

private slots:
    void onTcpServerStarted(QString listenIp, quint16 port);
    void onTcpServerStopped();
    void onTcpCameraConnected(QString cameraIp, quint16 cameraPort);
    void onTcpCameraDisconnected(QString cameraIp);
    void onTcpHeartbeatReceived(QString cameraIp);
    void onTcpCommandReceived(QString cameraIp, QString command);
    void onTcpImageDataReceived(QString cameraIp, QByteArray imageData);
    void onTcpError(QString errorMessage);

    void onFtpMonitorStarted(QString directory);
    void onFtpMonitorStopped();
    void onFtpNewImageDetected(ImageFileInfo imageInfo);
    void onFtpImageReady(ImageFileInfo imageInfo);
    void onFtpError(QString errorMessage);

    void onTestCaptureTimer();

private:
    struct FtpBinding {
        QString cameraIp;
        QString ftpDirectory;
        HikSmartCameraFtpMonitor* monitor = nullptr;
    };

    static void registerMetaTypes();
    void setState(HikCameraCState state, const QString& description);
    void initializeTcpServer();
    void cleanupTcpServer();
    void initializeFtpMonitors();
    void cleanupFtpMonitors();
    bool saveImageToFile(const QByteArray& imageData, CaptureType type, const QString& cameraIp);
    QString getCaptureTypeString(CaptureType type) const;
    bool isConfiguredCameraIp(const QString& cameraIp) const;
    QString groupLabelForCamera(const QString& cameraIp) const;
    QString formatConfiguredCameraList() const;
    void reportConfiguredCameraConnections() const;
    void scheduleStartupConnectionReport();
    void updateReadyStateFromConnections();
    void scheduleReadyStateUpdate();
    FtpBinding* findFtpBindingByMonitor(HikSmartCameraFtpMonitor* monitor);

    HikSmartCameraTcpServer* m_tcpServer = nullptr;
    QVector<FtpBinding> m_ftpBindings;
    QTimer* m_testCaptureTimer = nullptr;
    scan_tracking::common::VisionConfig m_config;
    bool m_started = false;
    HikCameraCState m_state = HikCameraCState::Idle;
    QStringList m_configuredCameraIps;
    QString m_primaryCameraIp;
    QString m_tcpListenIp;
    quint16 m_tcpListenPort = 0;
    QHash<QString, CaptureType> m_pendingCaptureTypeByIp;
    QHash<QString, int> m_captureCounterByIp;

    QString m_lastOcrText;
    QString m_lastOcrCameraIp;
    QString m_lastLoggedOcrText;
    qint64 m_lastOcrLogMs = 0;
    int m_suppressedOcrLogCount = 0;
};

}  // namespace vision
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::vision::HikCameraCState)
