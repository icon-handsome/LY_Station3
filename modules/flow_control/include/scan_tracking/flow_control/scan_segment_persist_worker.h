#pragma once

#include "scan_tracking/flow_control/scan_segment_cache.h"

#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

namespace scan_tracking::flow_control {

struct ScanSegmentPersistJob {
    QString runRoot;
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int segmentIndex = 0;
    quint32 taskId = 0;
    QString captureTimestamp;
    vision::MultiCameraCaptureBundle bundle;
    QString triggerLabel;
    std::shared_ptr<std::promise<void>> completion;
};

/// 单线程队列：扫描段 PLY/PNG/BMP 落盘不阻塞主线程与 PLC ACK。
class ScanSegmentPersistWorker {
public:
    ScanSegmentPersistWorker();
    ~ScanSegmentPersistWorker();

    ScanSegmentPersistWorker(const ScanSegmentPersistWorker&) = delete;
    ScanSegmentPersistWorker& operator=(const ScanSegmentPersistWorker&) = delete;

    using PersistFinishedHandler =
        std::function<void(common::ScanDeviceKind device, int segmentIndex, bool ok)>;

    void setPersistFinishedHandler(PersistFinishedHandler handler);
    /// 返回的 future 在该任务完成落盘后就绪；调用方可在非 UI 线程建立算法启动屏障。
    std::shared_future<void> enqueue(ScanSegmentPersistJob job);
    /// stop/析构前 drain 队列并 join；可重复调用。
    void stopAndJoin();
    /// CmdStart 等重启后恢复可投递状态。
    void restart();

private:
    void ensureWorkerRunning();
    void workerLoop();

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<ScanSegmentPersistJob> m_queue;
    std::thread m_worker;
    PersistFinishedHandler m_finishedHandler;
    bool m_stopping = false;
    bool m_workerRunning = false;
};

}  // namespace scan_tracking::flow_control
