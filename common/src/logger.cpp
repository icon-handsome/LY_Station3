#include "scan_tracking/common/logger.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QMutexLocker>
#include <QtCore/QReadLocker>
#include <QtCore/QWriteLocker>

#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cstdio>
#endif

namespace scan_tracking::common {

namespace {

constexpr char kUtf8Bom[] = "\xEF\xBB\xBF";

thread_local int g_log_handler_depth = 0;

const char* safeCategoryName(const QMessageLogContext& context)
{
    if (context.category != nullptr && context.category[0] != '\0') {
        return context.category;
    }
    return "default";
}

QString instanceLogFilePath(const QString& log_dir, const QDateTime& started_at)
{
    return QDir(log_dir).filePath(
        QStringLiteral("scan_tracking_%1.txt")
            .arg(started_at.toString(QStringLiteral("yyyy-MM-dd_HHmmss_zzz"))));
}

QString instanceAlgorithmLogFilePath(const QString& log_dir, const QDateTime& started_at)
{
    return QDir(log_dir).filePath(
        QStringLiteral("algorithm_scan_tracking_%1.txt")
            .arg(started_at.toString(QStringLiteral("yyyy-MM-dd_HHmmss_zzz"))));
}

bool ensureLogDirectory(const QString& log_dir)
{
    if (log_dir.isEmpty()) {
        return false;
    }
    QDir dir(log_dir);
    return (dir.exists() || dir.mkpath(QStringLiteral("."))) && dir.exists();
}

void writeConsoleLine(QtMsgType type, const char* data, std::size_t size)
{
#ifdef _WIN32
    const DWORD streamId =
        (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
        ? STD_ERROR_HANDLE
        : STD_OUTPUT_HANDLE;
    const HANDLE handle = GetStdHandle(streamId);
    if (handle != nullptr && handle != INVALID_HANDLE_VALUE) {
        DWORD written = 0;
        WriteFile(handle, data, static_cast<DWORD>(size), &written, nullptr);
        WriteFile(handle, "\r\n", 2, &written, nullptr);
    }
#else
    FILE* out = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) ? stderr : stdout;
    std::fwrite(data, 1, size, out);
    std::fwrite("\n", 1, 1, out);
    std::fflush(out);
#endif
}

QByteArray buildLogLine(
    const char* time_stamp,
    const char* severity,
    const char* category,
    const char* msg_utf8,
    const char* source_suffix)
{
    QByteArray line;
    line += '[';
    line += time_stamp;
    line += "] [";
    line += severity;
    line += "] [";
    line += category;
    line += "] ";
    line += msg_utf8;
    if (source_suffix != nullptr && source_suffix[0] != '\0') {
        line += source_suffix;
    }
    return line;
}

QByteArray sourceLocationSuffix(const QMessageLogContext& context)
{
    if (context.file == nullptr || context.line <= 0) {
        return {};
    }

    QByteArray suffix(" (");
    suffix += context.file;
    suffix += ':';
    suffix += QByteArray::number(context.line);
    suffix += ')';
    return suffix;
}

void emitMinimalFallback(QtMsgType type, const QMessageLogContext& context, const QByteArray& msg_utf8)
{
    const char* severity = "UNK";
    switch (type) {
        case QtDebugMsg: severity = "DBG"; break;
        case QtInfoMsg: severity = "INF"; break;
        case QtWarningMsg: severity = "WRN"; break;
        case QtCriticalMsg: severity = "CRT"; break;
        case QtFatalMsg: severity = "FTL"; break;
        default: break;
    }
    const QByteArray suffix = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
        ? sourceLocationSuffix(context)
        : QByteArray{};
    const QByteArray line = buildLogLine(
        "reentrant",
        severity,
        safeCategoryName(context),
        msg_utf8.constData(),
        suffix.isEmpty() ? nullptr : suffix.constData());
    writeConsoleLine(type, line.constData(), static_cast<std::size_t>(line.size()));
}

QFile* openLogFileWithBom(const QString& path)
{
    auto* file = new QFile(path);
    if (!file->open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        const QByteArray message = QStringLiteral("严重错误：Logger 无法打开目标文件：%1")
                                       .arg(path)
                                       .toUtf8();
        writeConsoleLine(QtCriticalMsg, message.constData(), message.size());
        delete file;
        return nullptr;
    }
    file->write(kUtf8Bom, 3);
    file->flush();
    return file;
}

}  // namespace

Logger* Logger::instance_ = nullptr;
QtMessageHandler Logger::previous_handler_ = nullptr;
QReadWriteLock Logger::instance_lock_(QReadWriteLock::Recursive);

Logger::Logger(const QString& log_dir)
    : log_dir_(log_dir),
      min_level_(QtDebugMsg)
{
    if (!ensureLogDirectory(log_dir_)) {
        const QByteArray message = QStringLiteral("严重错误：Logger 无法创建日志目录：%1")
                                       .arg(log_dir_)
                                       .toUtf8();
        writeConsoleLine(QtCriticalMsg, message.constData(), message.size());
    }
    openLogFile();
}

Logger::~Logger()
{
    if (log_file_ != nullptr) {
        log_file_->close();
        delete log_file_;
        log_file_ = nullptr;
    }
    if (algorithm_log_file_ != nullptr) {
        algorithm_log_file_->close();
        delete algorithm_log_file_;
        algorithm_log_file_ = nullptr;
    }
}

void Logger::initialize(const QString& log_dir)
{
    QWriteLocker instance_guard(&instance_lock_);
    if (instance_ != nullptr) {
        return;
    }

    instance_ = new Logger(log_dir);
    previous_handler_ = qInstallMessageHandler(Logger::messageHandler);
}

void Logger::cleanup()
{
    QWriteLocker instance_guard(&instance_lock_);
    if (instance_ == nullptr) {
        return;
    }

    QtMessageHandler upstream = previous_handler_;
    qInstallMessageHandler(upstream);
    previous_handler_ = nullptr;

    Logger* doomed = instance_;
    instance_ = nullptr;

    delete doomed;
}

Logger* Logger::instance()
{
    QReadLocker instance_guard(&instance_lock_);
    return instance_;
}

void Logger::setMinLevel(QtMsgType level)
{
    QMutexLocker lock(&mutex_);
    min_level_ = level;
}

int Logger::getSeverityLevel(QtMsgType type)
{
    switch (type) {
        case QtDebugMsg: return 0;
        case QtInfoMsg: return 1;
        case QtWarningMsg: return 2;
        case QtCriticalMsg: return 3;
        case QtFatalMsg: return 4;
        default: return 0;
    }
}

const char* Logger::getLogSeverity(QtMsgType type)
{
    switch (type) {
        case QtDebugMsg: return "DBG";
        case QtInfoMsg: return "INF";
        case QtWarningMsg: return "WRN";
        case QtCriticalMsg: return "CRT";
        case QtFatalMsg: return "FTL";
        default: return "UNK";
    }
}

bool Logger::isAlgorithmLogCategory(const char* category)
{
    if (category == nullptr || category[0] == '\0') {
        return false;
    }

    // 算法编排入口（AutoInspection / Trig_Inspection 解算生命周期）
    if (std::strcmp(category, "algorithm") == 0) {
        return true;
    }

    // 第三工位检测编排（各路径统一）
    if (std::strcmp(category, "flow_control.station3_inspection") == 0) {
        return true;
    }

    // weld / length_volume / thickness / inner_surface / undercut_length 等测量服务
    static constexpr char kMeasureServiceSuffix[] = "_measure.service";
    const std::size_t catLen = std::strlen(category);
    const std::size_t sufLen = sizeof(kMeasureServiceSuffix) - 1;
    return catLen >= sufLen
        && std::strcmp(category + (catLen - sufLen), kMeasureServiceSuffix) == 0;
}

void Logger::writeLineToFile(QFile* file, const QByteArray& line)
{
    if (file == nullptr) {
        return;
    }
    file->write(line);
    file->write("\r\n", 2);
    file->flush();
}

void Logger::openLogFile()
{
    const QDateTime started_at = QDateTime::currentDateTime();
    log_file_path_ = instanceLogFilePath(log_dir_, started_at);
    algorithm_log_file_path_ = instanceAlgorithmLogFilePath(log_dir_, started_at);

    if (log_file_ != nullptr) {
        log_file_->close();
        delete log_file_;
        log_file_ = nullptr;
    }
    if (algorithm_log_file_ != nullptr) {
        algorithm_log_file_->close();
        delete algorithm_log_file_;
        algorithm_log_file_ = nullptr;
    }

    // 每次启动新文件；唯一时间戳避免覆盖历史实例；算法日志与全量日志共用时间戳。
    log_file_ = openLogFileWithBom(log_file_path_);
    algorithm_log_file_ = openLogFileWithBom(algorithm_log_file_path_);
}

void Logger::messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    QReadLocker instance_guard(&instance_lock_);
    Logger* logger = instance_;
    if (logger != nullptr) {
        logger->log(type, context, msg);
        return;
    }
    if (previous_handler_ != nullptr) {
        previous_handler_(type, context, msg);
    }
}

void Logger::log(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    const QByteArray msg_utf8 = msg.toUtf8();

    struct DepthGuard {
        DepthGuard() { ++g_log_handler_depth; }
        ~DepthGuard() { --g_log_handler_depth; }
    } depth_guard;

    if (g_log_handler_depth > 1) {
        emitMinimalFallback(type, context, msg_utf8);
        return;
    }

    QMutexLocker lock(&mutex_);

    if (getSeverityLevel(type) < getSeverityLevel(min_level_)) {
        return;
    }

    const QDateTime now = QDateTime::currentDateTime();
    const QByteArray time_stamp = now.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")).toUtf8();
    const char* category = safeCategoryName(context);
    const QByteArray suffix = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
        ? sourceLocationSuffix(context)
        : QByteArray{};
    const QByteArray line = buildLogLine(
        time_stamp.constData(),
        getLogSeverity(type),
        category,
        msg_utf8.constData(),
        suffix.isEmpty() ? nullptr : suffix.constData());

    writeLineToFile(log_file_, line);
    if (isAlgorithmLogCategory(category)) {
        writeLineToFile(algorithm_log_file_, line);
    }

    writeConsoleLine(type, line.constData(), static_cast<std::size_t>(line.size()));
}

}  // namespace scan_tracking::common
