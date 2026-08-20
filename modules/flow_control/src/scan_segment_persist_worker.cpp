#include "scan_tracking/flow_control/scan_segment_persist_worker.h"

#include "scan_tracking/common/config_manager.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>

Q_LOGGING_CATEGORY(LOG_SCAN_PERSIST, "flow_control.scan_persist")

namespace scan_tracking::flow_control {

ScanSegmentPersistWorker::ScanSegmentPersistWorker() = default;

ScanSegmentPersistWorker::~ScanSegmentPersistWorker()
{
    stopAndJoin();
}

void ScanSegmentPersistWorker::setPersistFinishedHandler(PersistFinishedHandler handler)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_finishedHandler = std::move(handler);
}

void ScanSegmentPersistWorker::restart()
{
    stopAndJoin();
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stopping = false;
}

void ScanSegmentPersistWorker::stopAndJoin()
{
    std::thread worker;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stopping = true;
        worker = std::move(m_worker);
    }
    m_cv.notify_all();
    if (worker.joinable()) {
        qInfo(LOG_SCAN_PERSIST).noquote() << QStringLiteral("等待扫描段落盘线程结束…");
        worker.join();
        qInfo(LOG_SCAN_PERSIST).noquote() << QStringLiteral("扫描段落盘线程已接合。");
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.clear();
        m_workerRunning = false;
    }
}

std::shared_future<void> ScanSegmentPersistWorker::enqueue(ScanSegmentPersistJob job)
{
    job.completion = std::make_shared<std::promise<void>>();
    std::shared_future<void> completion = job.completion->get_future().share();
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_stopping) {
            qWarning(LOG_SCAN_PERSIST).noquote()
                << QStringLiteral("落盘 worker 已停止，丢弃段")
                << job.segmentIndex
                << QStringLiteral(" taskId=") << job.taskId;
            job.completion->set_value();
            return completion;
        }
        // 软上限：扫描快于写盘时告警，便于发现内存堆积（不阻塞主线程）。
        constexpr std::size_t kSoftMaxPending = 4;
        if (m_queue.size() >= kSoftMaxPending) {
            qWarning(LOG_SCAN_PERSIST).noquote()
                << QStringLiteral("落盘队列积压 pending=") << static_cast<qint64>(m_queue.size())
                << QStringLiteral("（软上限 ") << static_cast<qint64>(kSoftMaxPending)
                << QStringLiteral("），磁盘可能跟不上扫描节奏");
        }
        m_queue.push_back(std::move(job));
        ensureWorkerRunning();
    }
    m_cv.notify_one();
    return completion;
}

void ScanSegmentPersistWorker::ensureWorkerRunning()
{
    if (m_workerRunning) {
        return;
    }
    m_worker = std::thread([this]() { workerLoop(); });
    m_workerRunning = true;
}

void ScanSegmentPersistWorker::workerLoop()
{
    for (;;) {
        ScanSegmentPersistJob job;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this]() { return m_stopping || !m_queue.empty(); });
            if (m_stopping && m_queue.empty()) {
                break;
            }
            job = std::move(m_queue.front());
            m_queue.pop_front();
        }

        QString persistError;
        const bool ok = persistScanSegmentBundle(
            job.runRoot,
            job.device,
            job.segmentIndex,
            job.taskId,
            job.captureTimestamp,
            job.bundle,
            &persistError);

        if (ok) {
            qInfo(LOG_SCAN_PERSIST).noquote()
                << job.triggerLabel << QStringLiteral("：后台落盘完成")
                << QStringLiteral(" device=")
                << common::ConfigManager::scanDeviceKindToString(job.device)
                << QStringLiteral(" 段=") << job.segmentIndex
                << QStringLiteral(" taskId=") << job.taskId
                << QStringLiteral(" runRoot=") << job.runRoot;
        } else {
            qWarning(LOG_SCAN_PERSIST).noquote()
                << job.triggerLabel << QStringLiteral("：后台落盘失败")
                << persistError
                << QStringLiteral(" device=")
                << common::ConfigManager::scanDeviceKindToString(job.device)
                << QStringLiteral(" 段=") << job.segmentIndex
                << QStringLiteral(" taskId=") << job.taskId
                << QStringLiteral(" runRoot=") << job.runRoot;
        }

        // 先放行等待落盘的算法线程；完成回调仍异步回投主线程做状态通知。
        if (job.completion) {
            job.completion->set_value();
        }

        PersistFinishedHandler finishedHandler;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            finishedHandler = m_finishedHandler;
        }
        if (finishedHandler) {
            const common::ScanDeviceKind device = job.device;
            const int segmentIndex = job.segmentIndex;
            const bool persistOk = ok;
            if (QObject* app = QCoreApplication::instance()) {
                QMetaObject::invokeMethod(
                    app,
                    [finishedHandler, device, segmentIndex, persistOk]() {
                        finishedHandler(device, segmentIndex, persistOk);
                    },
                    Qt::QueuedConnection);
            }
        }

        // 落盘完成后释放点云/图像缓冲，避免队列积压时内存叠加。
        job.bundle = vision::MultiCameraCaptureBundle{};
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_workerRunning = false;
}

}  // namespace scan_tracking::flow_control
