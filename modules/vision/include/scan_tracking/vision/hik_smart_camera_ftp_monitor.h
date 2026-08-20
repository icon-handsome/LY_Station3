#pragma once

// 海康智能相机 FTP 图像监控器
// 监控 FTP 上传目录，检测新图像文件并识别类型

#include <QtCore/QObject>
#include <QtCore/QFileSystemWatcher>
#include <QtCore/QMap>
#include <QtCore/QSet>
#include <QtCore/QTimer>
#include <QtCore/QDateTime>

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

// 图像文件信息
struct ImageFileInfo {
    QString filePath;           // 完整文件路径
    QString fileName;           // 文件名
    CaptureType captureType;    // 拍照类型
    qint64 fileSize;            // 文件大小（字节）
    QDateTime detectedTime;     // 检测到的时间
    QDateTime modifiedTime;     // 文件修改时间
    bool isComplete;            // 文件是否传输完成
};

class HikSmartCameraFtpMonitor : public QObject {
    Q_OBJECT

public:
    explicit HikSmartCameraFtpMonitor(QObject* parent = nullptr);
    ~HikSmartCameraFtpMonitor() override;

    // 启动监控
    bool start(const QString& ftpDirectory);
    void stop();

    bool isMonitoring() const { return m_monitoring; }
    QString monitoredDirectory() const { return m_ftpDirectory; }

    // 配置
    void setFileCheckInterval(int ms) { m_fileCheckIntervalMs = ms; }
    void setFileStableTime(int ms) { m_fileStableTimeMs = ms; }
    void setSupportedExtensions(const QStringList& extensions) { m_supportedExtensions = extensions; }

    // 统计信息
    int totalFilesDetected() const { return m_totalFilesDetected; }
    int filesInQueue() const { return m_pendingFiles.size(); }

signals:
    void monitorStarted(QString directory);
    void monitorStopped();
    void newImageDetected(ImageFileInfo imageInfo);
    void imageReady(ImageFileInfo imageInfo);  // 文件传输完成，可以处理
    void error(QString errorMessage);

private slots:
    void onDirectoryChanged(const QString& path);
    void onFileCheckTimer();

private:
    void scanDirectory();
    void checkPendingFiles();
    bool isImageFile(const QString& fileName) const;
    CaptureType detectCaptureType(const QString& fileName) const;
    bool isFileComplete(const QString& filePath, qint64& fileSize) const;
    void processNewFile(const QString& filePath);

    QFileSystemWatcher* m_watcher = nullptr;
    QTimer* m_fileCheckTimer = nullptr;
    
    QString m_ftpDirectory;
    bool m_monitoring = false;
    
    // 配置参数
    int m_fileCheckIntervalMs = 500;      // 文件检查间隔（毫秒）
    int m_fileStableTimeMs = 1000;        // 文件稳定时间（毫秒）
    QStringList m_supportedExtensions;    // 支持的文件扩展名
    
    // 文件跟踪
    QSet<QString> m_processedFiles;       // 已处理的文件
    QMap<QString, ImageFileInfo> m_pendingFiles;  // 待处理的文件
    
    // 统计
    int m_totalFilesDetected = 0;
};

}  // namespace vision
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::vision::ImageFileInfo)
