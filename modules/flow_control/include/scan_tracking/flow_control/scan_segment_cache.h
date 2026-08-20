#pragma once

#include <QMap>
#include <QString>
#include <QVector>

#include <QtCore/QtGlobal>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking::flow_control {

struct ScanSegmentCacheKey {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int localIndex = 0;

    bool operator<(const ScanSegmentCacheKey& other) const
    {
        if (device != other.device) {
            return static_cast<int>(device) < static_cast<int>(other.device);
        }
        return localIndex < other.localIndex;
    }

    bool operator==(const ScanSegmentCacheKey& other) const
    {
        return device == other.device && localIndex == other.localIndex;
    }
};

struct ScanSegmentCacheEntry {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int segmentIndex = 0;  ///< 设备内本地段号
    quint32 taskId = 0;
    QString runCaptureRoot;
    QString captureTimestamp;
    vision::MultiCameraCaptureBundle bundle;
};

class ScanSegmentCache {
public:
    /// 清空段缓存并丢弃当前 run 目录绑定（下次采集新建 run_*）。
    void reset();

    /// 仅清空段内存缓存，保留当前 run 落盘根目录（切路径时用，避免一次运行多个 run 文件夹）。
    void clearSegmentsKeepRunRoot();

    /// 剔除指定设备本地段（扫描失败策略=segment）；不存在则 no-op。
    bool removeSegment(common::ScanDeviceKind device, int localIndex);

    /// 为当前 taskId 准备 run 目录。taskId 变为「非 0 且与上次不同」时新建目录；
    /// taskId=0 时若已有 run 根则复用，保证整次运行只有一个文件夹。
    bool ensureRunRoot(quint32 taskId, QString* runRootOut = nullptr, QString* timestampOut = nullptr);

    /// 绑定已有 run 根（不 mkdir）。供后台解算物化临时缓存复用主实例落盘根。
    /// runRoot 为空时仅写入 taskId（若非 0），不新建目录。
    void bindExistingRunRoot(quint32 taskId, const QString& runRoot);

    void storeSegment(
        common::ScanDeviceKind device,
        int localIndex,
        quint32 taskId,
        vision::MultiCameraCaptureBundle&& bundle);

    /// prepareRunRoot=false：不解盘、不新建 run_*（临时评估缓存）。
    void storeSegment(
        common::ScanDeviceKind device,
        int localIndex,
        quint32 taskId,
        vision::MultiCameraCaptureBundle&& bundle,
        bool prepareRunRoot);

    bool persistSegment(
        common::ScanDeviceKind device,
        int localIndex,
        QString* errorMessage = nullptr);

    bool stripHeavyPayloads(common::ScanDeviceKind device, int localIndex);

    /// 算法与落盘任务都已通过 shared_ptr 接管后，释放缓存持有的主点云引用。
    /// 实际缓冲会在最后一个后台使用者结束时自动释放。
    bool releaseSegmentPointCloud(common::ScanDeviceKind device, int localIndex);

    const ScanSegmentCacheEntry* entry(common::ScanDeviceKind device, int localIndex) const;
    QVector<ScanSegmentCacheKey> cachedKeys() const;
    bool allCachedBundlesSuccessful() const;
    int cachedSegmentCount() const;
    int cachedCountForDevice(common::ScanDeviceKind device) const;
    bool meetsDeviceQuotas(int expectedArmCount, int expectedTelescopicCount) const;
    quint32 runTaskId() const { return m_runTaskId; }
    const QString& runCaptureRoot() const { return m_runCaptureRoot; }

private:
    QMap<ScanSegmentCacheKey, ScanSegmentCacheEntry> m_entries;
    quint32 m_runTaskId = 0;
    QString m_runCaptureRoot;
    QString m_runTimestamp;
};

bool persistScanSegmentBundle(
    const QString& runRoot,
    common::ScanDeviceKind device,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp,
    const vision::MultiCameraCaptureBundle& bundle,
    QString* errorMessage = nullptr);

/// 落盘 job 已持有缓冲副本后即可调用：剥离检测不需要的大块（保留 pointCloud）。
void stripScanSegmentHeavyPayloads(vision::MultiCameraCaptureBundle* bundle);

}  // namespace scan_tracking::flow_control
