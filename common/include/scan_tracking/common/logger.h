#pragma once

#include <QtCore/QString>
#include <QtCore/QMutex>
#include <QtCore/QReadWriteLock>
#include <QtCore/QtMessageHandler>

class QFile;

namespace scan_tracking::common {

// 每次进程启动新建：
//   logs/scan_tracking_yyyy-MM-dd_HHmmss_zzz.txt              （全量运行日志）
//   logs/algorithm_scan_tracking_yyyy-MM-dd_HHmmss_zzz.txt    （算法日志，同时间戳）
// 同一次运行始终写入这对文件，不按自然日切换。
class Logger {
public:
    static void initialize(const QString& log_dir = QStringLiteral("logs"));
    static void cleanup();

    static void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static Logger* instance();

    void setMinLevel(QtMsgType level);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    explicit Logger(const QString& log_dir);
    ~Logger();

    void openLogFile();
    void log(QtMsgType type, const QMessageLogContext& context, const QString& msg);
    static bool isAlgorithmLogCategory(const char* category);
    static void writeLineToFile(QFile* file, const QByteArray& line);

    static const char* getLogSeverity(QtMsgType type);
    static int getSeverityLevel(QtMsgType type);

    QString log_dir_;
    QString log_file_path_;
    QString algorithm_log_file_path_;
    QFile* log_file_ = nullptr;
    QFile* algorithm_log_file_ = nullptr;
    QMutex mutex_;
    QtMsgType min_level_;

    static Logger* instance_;
    static QtMessageHandler previous_handler_;
    static QReadWriteLock instance_lock_;
};

}  // namespace scan_tracking::common
