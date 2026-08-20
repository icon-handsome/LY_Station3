#pragma once

/**
 * @file orbbec_gemini_service.h
 * @brief Orbbec Gemini 深度相机服务（主线程门面）
 *
 * 对外提供线程安全的设备管理与采集接口：
 * - 在独立 QThread 中运行 OrbbecGeminiWorker，避免阻塞 UI/状态机
 * - 转发 Worker 信号为对外公开信号
 * - 管理采集忙状态与请求 ID 分配
 *
 * 典型用法：start() → 等待 openFinished/stateChanged(Ready) → requestCapture() → captureFinished
 */

#include <QtCore/QObject>

#include "scan_tracking/orbbec_gemini/orbbec_gemini_types.h"

QT_BEGIN_NAMESPACE
class QThread;
QT_END_NAMESPACE

namespace scan_tracking {
namespace orbbec_gemini {

class OrbbecGeminiWorker;

class OrbbecGeminiService : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父对象
     */
    explicit OrbbecGeminiService(QObject* parent = nullptr);

    /**
     * @brief 析构函数，自动调用 stop() 停止工作线程
     */
    ~OrbbecGeminiService() override;

    /**
     * @brief 启动服务：创建工作线程并异步打开 Orbbec 设备
     *
     * 若未通过 setOpenConfig 显式设置配置，则从 ConfigManager 读取 orbbecGeminiConfig。
     * 重复调用 start() 在无操作的情况下直接返回。
     */
    void start();

    /**
     * @brief 停止服务：同步停止 Worker 并等待工作线程退出（最多 10 秒）
     */
    void stop();

    /**
     * @brief 设置设备打开配置（须在 start() 之前调用才生效）
     * @param config 打开参数、流参数与缓存目录等
     */
    void setOpenConfig(const OrbbecGeminiOpenConfig& config);

    /// 当前运行时状态（与 Worker 同步，经 onWorkerStateChanged 更新）
    OrbbecGeminiRuntimeState state() const { return m_currentState; }

    /// 是否正在执行采集（Capturing 期间为 true）
    bool isBusy() const { return m_busy; }

    /**
     * @brief 请求采集一帧深度图
     *
     * 仅在服务已启动、未停止、状态为 Ready 且非 busy 时接受请求。
     *
     * @param timeoutMs 等待深度帧超时（毫秒）；0 表示使用配置中的 captureTimeoutMs
     * @param saveToDisk 是否落盘；最终是否保存还受 openConfig.saveCaptureToDisk 约束
     * @return 非零 requestId 表示请求已投递；返回 0 表示当前无法接受请求
     */
    quint64 requestCapture(int timeoutMs = 0, bool saveToDisk = true);

signals:
    /// 设备枚举完成（由 Worker 转发）
    void enumerateFinished(QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary> devices);

    /**
     * @brief 设备打开完成
     * @param success 是否成功
     * @param deviceInfo 已打开设备信息
     * @param errorMessage 失败时的错误描述
     */
    void openFinished(
        bool success,
        scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary deviceInfo,
        QString errorMessage);

    /**
     * @brief 运行时状态变化
     * @param newState 新状态
     * @param description 状态说明
     */
    void stateChanged(
        scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState newState,
        QString description);

    /// 单次采集完成
    void captureFinished(scan_tracking::orbbec_gemini::OrbbecCaptureResult result);

    /// 日志消息（由 Worker 转发）
    void logMessage(QString message);

private slots:
    /// Worker 枚举完成 → 转发 enumerateFinished
    void onWorkerEnumerateFinished(
        QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary> devices);

    /// Worker 打开完成 → 更新状态并转发 openFinished
    void onWorkerOpenFinished(
        bool success,
        scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary deviceInfo,
        QString errorMessage);

    /// Worker 状态变化 → 同步 m_currentState/m_busy 并转发
    void onWorkerStateChanged(
        scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState newState,
        QString description);

    /// Worker 采集完成 → 清除 busy 并转发 captureFinished
    void onWorkerCaptureFinished(scan_tracking::orbbec_gemini::OrbbecCaptureResult result);

    /// Worker 日志 → 转发 logMessage
    void onWorkerLogMessage(QString message);

signals:
    /// 内部信号：通知 Worker 线程启动（QueuedConnection → startWorker）
    void sig_startWorker(scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig config);

    /// 内部信号：通知 Worker 线程停止
    void sig_stopWorker();

    /// 内部信号：通知 Worker 执行采集
    void sig_performCapture(scan_tracking::orbbec_gemini::OrbbecCaptureRequest request);

private:
    /// 注册本模块自定义类型到 Qt 元对象系统（跨线程信号槽必需）
    static void registerMetaTypes();

    QThread* m_workerThread = nullptr;
    OrbbecGeminiWorker* m_worker = nullptr;
    OrbbecGeminiOpenConfig m_openConfig;
    int m_defaultCaptureTimeoutMs = 5000;
    quint64 m_nextRequestId = 1;
    OrbbecGeminiRuntimeState m_currentState = OrbbecGeminiRuntimeState::Idle;
    bool m_started = false;
    bool m_stopping = false;
    bool m_busy = false;
    bool m_hasExplicitOpenConfig = false;
};

}  // namespace orbbec_gemini
}  // namespace scan_tracking
