#include "scan_tracking/vision/hik_smart_camera_ftp_monitor.h"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

Q_LOGGING_CATEGORY(hikFtpMonitorLog, "vision.hik_ftp_monitor")

namespace scan_tracking {
namespace vision {

HikSmartCameraFtpMonitor::HikSmartCameraFtpMonitor(QObject* parent)
    : QObject(parent)
    , m_watcher(new QFileSystemWatcher(this))
    , m_fileCheckTimer(new QTimer(this))
{
    // 默认支持的图像格式
    m_supportedExtensions << "jpg" << "jpeg" << "png" << "bmp" << "tif" << "tiff";

    connect(m_watcher, &QFileSystemWatcher::directoryChanged,
            this, &HikSmartCameraFtpMonitor::onDirectoryChanged);

    m_fileCheckTimer->setInterval(m_fileCheckIntervalMs);
    connect(m_fileCheckTimer, &QTimer::timeout,
            this, &HikSmartCameraFtpMonitor::onFileCheckTimer);
}

HikSmartCameraFtpMonitor::~HikSmartCameraFtpMonitor()
{
    stop();
}

bool HikSmartCameraFtpMonitor::start(const QString& ftpDirectory)
{
    if (m_monitoring) {
        qWarning(hikFtpMonitorLog) << QStringLiteral("监控器已在运行");
        return false;
    }

    // 检查目录是否存在；不存在则尝试创建（现场常缺 D:/HikCameraFTP）
    QDir dir(ftpDirectory);
    if (!dir.exists()) {
        if (!QDir().mkpath(ftpDirectory)) {
            qCritical(hikFtpMonitorLog) << QStringLiteral("FTP 目录不存在且创建失败：") << ftpDirectory;
            emit error(QStringLiteral("FTP 目录不存在且创建失败: %1").arg(ftpDirectory));
            return false;
        }
        dir = QDir(ftpDirectory);
        qInfo(hikFtpMonitorLog) << QStringLiteral("FTP 目录已自动创建：") << dir.absolutePath();
    }

    m_ftpDirectory = dir.absolutePath();
    
    // 添加目录到监控
    if (!m_watcher->addPath(m_ftpDirectory)) {
        qCritical(hikFtpMonitorLog) << QStringLiteral("无法将目录加入监控：") << m_ftpDirectory;
        emit error(QStringLiteral("无法监控目录: %1").arg(m_ftpDirectory));
        return false;
    }

    m_monitoring = true;
    m_totalFilesDetected = 0;
    m_processedFiles.clear();
    m_pendingFiles.clear();

    // 启动定时检查
    m_fileCheckTimer->start();

    // 把启动前已存在的文件全部标记为已处理，只监控后续新增文件
    // 避免历史文件触发大量 imageReady 信号，也防止启动时的信号风暴
    {
        QDir dir(m_ftpDirectory);
        const QFileInfoList existingFiles = dir.entryInfoList(
            QDir::Files | QDir::NoDotAndDotDot, QDir::Time);
        for (const QFileInfo& fi : existingFiles) {
            if (isImageFile(fi.fileName())) {
                m_processedFiles.insert(fi.absoluteFilePath());
            }
        }
        qInfo(hikFtpMonitorLog) << "启动时跳过已有图片文件：" << m_processedFiles.size() << "个";
    }

    qInfo(hikFtpMonitorLog) << "FTP 监控器已启动，监控目录：" << m_ftpDirectory;
    emit monitorStarted(m_ftpDirectory);
    return true;
}

void HikSmartCameraFtpMonitor::stop()
{
    if (!m_monitoring) {
        return;
    }

    m_monitoring = false;
    m_fileCheckTimer->stop();

    if (!m_ftpDirectory.isEmpty()) {
        m_watcher->removePath(m_ftpDirectory);
    }

    m_pendingFiles.clear();
    qInfo(hikFtpMonitorLog) << "FTP 监控器已停止";
    emit monitorStopped();
}

void HikSmartCameraFtpMonitor::onDirectoryChanged(const QString& path)
{
    qDebug(hikFtpMonitorLog) << QStringLiteral("目录变化：") << path;
    scanDirectory();
}

void HikSmartCameraFtpMonitor::onFileCheckTimer()
{
    checkPendingFiles();
}

void HikSmartCameraFtpMonitor::scanDirectory()
{
    if (!m_monitoring) {
        return;
    }

    QDir dir(m_ftpDirectory);
    QFileInfoList fileList = dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot, QDir::Time);

    for (const QFileInfo& fileInfo : fileList) {
        QString filePath = fileInfo.absoluteFilePath();
        QString fileName = fileInfo.fileName();

        // 跳过已处理的文件
        if (m_processedFiles.contains(filePath)) {
            continue;
        }

        // 跳过非图像文件
        if (!isImageFile(fileName)) {
            continue;
        }

        // 处理新文件
        processNewFile(filePath);
    }
}

void HikSmartCameraFtpMonitor::processNewFile(const QString& filePath)
{
    QFileInfo fileInfo(filePath);
    QString fileName = fileInfo.fileName();

    // 检查是否已在待处理队列
    if (m_pendingFiles.contains(filePath)) {
        return;
    }

    // 识别拍照类型
    CaptureType captureType = detectCaptureType(fileName);

    // 创建文件信息
    ImageFileInfo imageInfo;
    imageInfo.filePath = filePath;
    imageInfo.fileName = fileName;
    imageInfo.captureType = captureType;
    imageInfo.fileSize = 0;
    imageInfo.detectedTime = QDateTime::currentDateTime();
    imageInfo.modifiedTime = fileInfo.lastModified();
    imageInfo.isComplete = false;

    // 添加到待处理队列
    m_pendingFiles[filePath] = imageInfo;
    m_totalFilesDetected++;

    emit newImageDetected(imageInfo);
}

void HikSmartCameraFtpMonitor::checkPendingFiles()
{
    if (m_pendingFiles.isEmpty()) {
        return;
    }

    QStringList completedFiles;

    for (auto it = m_pendingFiles.begin(); it != m_pendingFiles.end(); ++it) {
        QString filePath = it.key();
        ImageFileInfo& imageInfo = it.value();

        qint64 currentSize = 0;
        if (!isFileComplete(filePath, currentSize)) {
            // 文件不存在或无法访问
            qWarning(hikFtpMonitorLog) << QStringLiteral("文件不可访问：") << filePath;
            completedFiles.append(filePath);
            m_processedFiles.insert(filePath);
            continue;
        }

        // 检查文件大小是否稳定
        if (imageInfo.fileSize == 0) {
            // 第一次检查，记录大小
            imageInfo.fileSize = currentSize;
            continue;
        }

        if (imageInfo.fileSize == currentSize) {
            // 文件大小稳定，检查时间
            qint64 elapsedMs = imageInfo.detectedTime.msecsTo(QDateTime::currentDateTime());
            if (elapsedMs >= m_fileStableTimeMs) {
                // 文件传输完成
                imageInfo.isComplete = true;
                imageInfo.fileSize = currentSize;
                
                emit imageReady(imageInfo);
                completedFiles.append(filePath);
                m_processedFiles.insert(filePath);
            }
        } else {
            // 文件大小变化，重新计时
            imageInfo.fileSize = currentSize;
            imageInfo.detectedTime = QDateTime::currentDateTime();
        }
    }

    // 移除已完成的文件
    for (const QString& filePath : completedFiles) {
        m_pendingFiles.remove(filePath);
    }
}

bool HikSmartCameraFtpMonitor::isImageFile(const QString& fileName) const
{
    QFileInfo fileInfo(fileName);
    QString extension = fileInfo.suffix().toLower();
    return m_supportedExtensions.contains(extension);
}

CaptureType HikSmartCameraFtpMonitor::detectCaptureType(const QString& fileName) const
{
    QString lowerName = fileName.toLower();

    // 根据文件名关键字识别类型
    // 可以根据实际相机命名规则调整
    
    if (lowerName.contains("surface") || lowerName.contains("defect") || 
        lowerName.contains("表面") || lowerName.contains("缺陷")) {
        return CaptureType::SurfaceDefect;
    }
    
    if (lowerName.contains("weld") || lowerName.contains("焊缝") || 
        lowerName.contains("焊接")) {
        return CaptureType::WeldDefect;
    }
    
    if (lowerName.contains("number") || lowerName.contains("ocr") || 
        lowerName.contains("编号") || lowerName.contains("识别")) {
        return CaptureType::NumberRecognition;
    }

    // 默认为表面缺陷
    return CaptureType::SurfaceDefect;
}

bool HikSmartCameraFtpMonitor::isFileComplete(const QString& filePath, qint64& fileSize) const
{
    QFileInfo fileInfo(filePath);
    
    if (!fileInfo.exists()) {
        return false;
    }

    // 尝试以只读方式打开文件，检查是否可访问
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    file.close();

    fileSize = fileInfo.size();
    return true;
}

}  // namespace vision
}  // namespace scan_tracking
