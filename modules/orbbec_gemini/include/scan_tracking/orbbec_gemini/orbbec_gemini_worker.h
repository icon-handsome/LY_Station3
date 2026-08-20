#pragma once

/**
 * @file orbbec_gemini_worker.h
 * @brief Orbbec Gemini 深度相机工作对象（运行于独立线程）
 *
 * 封装 libobsensor SDK，在后台线程中完成设备枚举、打开、深度流启动、
 * 单帧采集与点云生成。通过 Qt 信号将结果回传到主线程的 OrbbecGeminiService。
 *
 * 注意：本类的 public slots 必须通过 QueuedConnection 从其他线程调用。
 */

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/orbbec_gemini/orbbec_gemini_types.h"

namespace scan_tracking {
namespace orbbec_gemini {

class OrbbecGeminiWorker : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父对象
     */
    explicit OrbbecGeminiWorker(QObject* parent = nullptr);

    /**
     * @brief 析构函数，自动调用 stopWorker() 释放 SDK 资源
     */
    ~OrbbecGeminiWorker() override;

public slots:
    /**
     * @brief 启动工作流：枚举设备 → 打开设备 → 启动深度流 → 预热丢帧
     *
     * 若已有打开的 pipeline，会先调用 stopWorker() 再重新初始化。
     * 完成后发出 enumerateFinished、openFinished 与 stateChanged 信号。
     *
     * @param config 设备打开与流配置；serial 非空时按序列号打开，否则按 deviceIndex
     */
    void startWorker(const scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig& config);

    /**
     * @brief 停止深度流并释放 Context/Device/Pipeline
     *
     * 发出 stateChanged(Stopped) 信号。正在进行的采集会在 performCapture 入口提前返回 NotReady。
     */
    void stopWorker();

    /**
     * @brief 采集一帧深度图，可选生成点云并落盘
     *
     * 要求深度流已启动（state 为 Ready）。采集期间发出 stateChanged(Capturing)，
     * 结束后通过 captureFinished 返回 OrbbecCaptureResult。
     *
     * @param request 采集请求，含 requestId、超时与是否落盘
     */
    void performCapture(const scan_tracking::orbbec_gemini::OrbbecCaptureRequest& request);

signals:
    /// 设备枚举完成，返回所有可探测到的设备摘要（可能为空列表）
    void enumerateFinished(QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary> devices);

    /**
     * @brief 设备打开完成
     * @param success 是否成功启动深度流
     * @param deviceInfo 成功时为已打开设备信息，失败时字段为空
     * @param errorMessage 失败原因描述
     */
    void openFinished(
        bool success,
        scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary deviceInfo,
        QString errorMessage);

    /**
     * @brief 运行时状态变化
     * @param newState 新状态
     * @param description 状态说明（用于日志/UI 展示）
     */
    void stateChanged(
        scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState newState,
        QString description);

    /// 单次采集完成（成功或失败均会发出）
    void captureFinished(scan_tracking::orbbec_gemini::OrbbecCaptureResult result);

    /// 工作线程日志消息，供上层转发或记录
    void logMessage(QString message);

private:
    class Impl;  // PIMPL，持有 ob::Context/Device/Pipeline 等 SDK 对象
    Impl* m_impl = nullptr;
    OrbbecGeminiOpenConfig m_config;       // 最近一次 startWorker 使用的配置
    OrbbecGeminiDeviceSummary m_openedDevice;  // 当前已打开设备的摘要
    bool m_stopping = false;               // 是否正在停止，用于拒绝新的采集
};

}  // namespace orbbec_gemini
}  // namespace scan_tracking
