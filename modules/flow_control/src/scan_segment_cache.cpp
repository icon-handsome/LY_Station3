#include "scan_tracking/flow_control/scan_segment_cache.h"

#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/vision/hik_mono_io.h"
#include "scan_tracking/vision/lb_pose_io.h"

#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>

#include <algorithm>
#include <utility>

Q_LOGGING_CATEGORY(LOG_SCAN_CACHE, "flow_control.scan_cache")

namespace scan_tracking::flow_control {

namespace {

QString deviceTagForPath(common::ScanDeviceKind device)
{
    return common::ConfigManager::scanDeviceKindToString(device);
}

}  // namespace

void ScanSegmentCache::reset()
{
    m_entries.clear();
    m_runTaskId = 0;
    m_runCaptureRoot.clear();
    m_runTimestamp.clear();
}

void ScanSegmentCache::clearSegmentsKeepRunRoot()
{
    m_entries.clear();
}

bool ScanSegmentCache::removeSegment(common::ScanDeviceKind device, int localIndex)
{
    const ScanSegmentCacheKey key{device, localIndex};
    return m_entries.remove(key) > 0;
}

bool ScanSegmentCache::ensureRunRoot(quint32 taskId, QString* runRootOut, QString* timestampOut)
{
    // 仅当 PLC 给出明确且变化的非 0 taskId 时，才换新的运行实例目录。
    // taskId=0（未写）全程复用已有 run_*，避免切路径后反复新建文件夹。
    const bool switchToNewTask =
        taskId != 0 && m_runTaskId != 0 && taskId != m_runTaskId;
    if (switchToNewTask) {
        m_entries.clear();
        m_runTaskId = taskId;
        m_runTimestamp.clear();
        m_runCaptureRoot.clear();
    } else if (taskId != 0) {
        m_runTaskId = taskId;
    }

    if (m_runCaptureRoot.isEmpty()) {
        if (m_runTimestamp.isEmpty()) {
            m_runTimestamp = scan_tracking::common::buildCaptureTimestamp();
        }
        if (taskId != 0) {
            m_runTaskId = taskId;
        }
        m_runCaptureRoot =
            scan_tracking::common::buildRunCaptureRoot(m_runTaskId, m_runTimestamp);
        if (m_runCaptureRoot.isEmpty()) {
            qWarning(LOG_SCAN_CACHE).noquote()
                << QStringLiteral("创建 run 落盘目录失败 taskId=") << m_runTaskId
                << QStringLiteral(" timestamp=") << m_runTimestamp;
            if (runRootOut != nullptr) {
                *runRootOut = QString();
            }
            if (timestampOut != nullptr) {
                *timestampOut = m_runTimestamp;
            }
            return false;
        }
        qInfo(LOG_SCAN_CACHE).noquote()
            << QStringLiteral("已创建运行实例落盘目录：") << m_runCaptureRoot
            << QStringLiteral(" taskId=") << m_runTaskId;
    }

    if (runRootOut != nullptr) {
        *runRootOut = m_runCaptureRoot;
    }
    if (timestampOut != nullptr) {
        *timestampOut = m_runTimestamp;
    }
    return true;
}

void ScanSegmentCache::bindExistingRunRoot(quint32 taskId, const QString& runRoot)
{
    if (taskId != 0) {
        m_runTaskId = taskId;
    }
    const QString root = runRoot.trimmed();
    if (!root.isEmpty()) {
        m_runCaptureRoot = root;
    }
}

void ScanSegmentCache::storeSegment(
    common::ScanDeviceKind device,
    int localIndex,
    quint32 taskId,
    vision::MultiCameraCaptureBundle&& bundle)
{
    storeSegment(device, localIndex, taskId, std::move(bundle), true);
}

void ScanSegmentCache::storeSegment(
    common::ScanDeviceKind device,
    int localIndex,
    quint32 taskId,
    vision::MultiCameraCaptureBundle&& bundle,
    bool prepareRunRoot)
{
    if (prepareRunRoot) {
        if (!ensureRunRoot(taskId)) {
            qWarning(LOG_SCAN_CACHE).noquote()
                << QStringLiteral("storeSegment：run 目录未就绪，段数据仅保留内存")
                << QStringLiteral(" device=")
                << deviceTagForPath(device)
                << QStringLiteral(" localIndex=") << localIndex
                << QStringLiteral(" taskId=") << taskId;
        }
    } else if (taskId != 0 && m_runTaskId == 0) {
        m_runTaskId = taskId;
    }

    ScanSegmentCacheKey key{device, localIndex};
    ScanSegmentCacheEntry entry;
    entry.device = device;
    entry.segmentIndex = localIndex;
    entry.taskId = taskId != 0 ? taskId : m_runTaskId;
    entry.runCaptureRoot = m_runCaptureRoot;
    entry.captureTimestamp = m_runTimestamp;
    entry.bundle = std::move(bundle);
    m_entries.insert(key, std::move(entry));
}

bool ScanSegmentCache::stripHeavyPayloads(common::ScanDeviceKind device, int localIndex)
{
    const ScanSegmentCacheKey key{device, localIndex};
    const auto iterator = m_entries.find(key);
    if (iterator == m_entries.end()) {
        return false;
    }

    const int pointsBefore = iterator->bundle.mechEyeResult.pointCloud.pointCount;
    stripScanSegmentHeavyPayloads(&iterator->bundle);
    qInfo(LOG_SCAN_CACHE).noquote()
        << QStringLiteral("段缓存已剥离重载荷（落盘 job 持有副本） device=")
        << deviceTagForPath(device)
        << QStringLiteral(" segment=") << localIndex
        << QStringLiteral(" 保留检测点云 points=") << pointsBefore;
    return true;
}

bool ScanSegmentCache::releaseSegmentPointCloud(
    common::ScanDeviceKind device,
    int localIndex)
{
    const ScanSegmentCacheKey key{device, localIndex};
    const auto iterator = m_entries.find(key);
    if (iterator == m_entries.end()) {
        return false;
    }

    auto& pointCloud = iterator->bundle.mechEyeResult.pointCloud;
    const int pointCount = pointCloud.pointCount;
    pointCloud = mech_eye::PointCloudFrame{};
    qInfo(LOG_SCAN_CACHE).noquote()
        << QStringLiteral("逐段算法与落盘已接管，释放段缓存主点云 device=")
        << deviceTagForPath(device)
        << QStringLiteral(" segment=") << localIndex
        << QStringLiteral(" points=") << pointCount;
    return true;
}

bool ScanSegmentCache::persistSegment(
    common::ScanDeviceKind device,
    int localIndex,
    QString* errorMessage)
{
    const ScanSegmentCacheKey key{device, localIndex};
    const auto iterator = m_entries.find(key);
    if (iterator == m_entries.end()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("%1 段 %2 不在缓存中。")
                                .arg(deviceTagForPath(device))
                                .arg(localIndex);
        }
        return false;
    }

    return persistScanSegmentBundle(
        iterator->runCaptureRoot,
        iterator->device,
        iterator->segmentIndex,
        iterator->taskId,
        iterator->captureTimestamp,
        iterator->bundle,
        errorMessage);
}

const ScanSegmentCacheEntry* ScanSegmentCache::entry(
    common::ScanDeviceKind device,
    int localIndex) const
{
    const auto iterator = m_entries.constFind(ScanSegmentCacheKey{device, localIndex});
    if (iterator == m_entries.constEnd()) {
        return nullptr;
    }
    return &(*iterator);
}

QVector<ScanSegmentCacheKey> ScanSegmentCache::cachedKeys() const
{
    QVector<ScanSegmentCacheKey> keys;
    keys.reserve(m_entries.size());
    for (auto iterator = m_entries.constBegin(); iterator != m_entries.constEnd(); ++iterator) {
        keys.push_back(iterator.key());
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

bool ScanSegmentCache::allCachedBundlesSuccessful() const
{
    for (auto iterator = m_entries.constBegin(); iterator != m_entries.constEnd(); ++iterator) {
        if (!iterator->bundle.success()) {
            return false;
        }
    }
    return !m_entries.isEmpty();
}

int ScanSegmentCache::cachedSegmentCount() const
{
    return m_entries.size();
}

int ScanSegmentCache::cachedCountForDevice(common::ScanDeviceKind device) const
{
    int count = 0;
    for (auto iterator = m_entries.constBegin(); iterator != m_entries.constEnd(); ++iterator) {
        if (iterator.key().device == device) {
            ++count;
        }
    }
    return count;
}

bool ScanSegmentCache::meetsDeviceQuotas(int expectedArmCount, int expectedTelescopicCount) const
{
    if (expectedArmCount > 0 &&
        cachedCountForDevice(common::ScanDeviceKind::Arm) < expectedArmCount) {
        return false;
    }
    if (expectedTelescopicCount > 0 &&
        cachedCountForDevice(common::ScanDeviceKind::Telescopic) < expectedTelescopicCount) {
        return false;
    }
    return cachedSegmentCount() > 0;
}

bool persistScanSegmentBundle(
    const QString& runRoot,
    common::ScanDeviceKind device,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp,
    const vision::MultiCameraCaptureBundle& bundle,
    QString* errorMessage)
{
    Q_UNUSED(timestamp);
    if (runRoot.trimmed().isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("run 落盘根目录为空（taskId=%1，请确认 output 可写）。")
                                .arg(taskId);
        }
        return false;
    }

    bool ok = true;
    QString firstError;
    const QString deviceLabel = deviceTagForPath(device);

    auto recordFailure = [&](const QString& message) {
        ok = false;
        if (firstError.isEmpty()) {
            firstError = message;
        }
    };

    const bool cxpParticipated =
        !bundle.request.hikCameraAKey.isEmpty() || !bundle.request.hikCameraBKey.isEmpty();

    int pathId = 0;
    if (const auto* configMgr = scan_tracking::common::ConfigManager::instance()) {
        pathId = configMgr->activePathId();
    }

    if (bundle.mechEyeResult.pointCloud.isValid() ||
        bundle.mechEyeResult.pointCloudRaw.isValid()) {
        const scan_tracking::mech_eye::GrayTextureFrame* texture =
            bundle.mechEyeResult.texture2D.isValid() ? &bundle.mechEyeResult.texture2D : nullptr;

        const bool hasRaw = bundle.mechEyeResult.pointCloudRaw.isValid();
        const bool lbStitched =
            bundle.lbPoseResult.invoked && bundle.lbPoseResult.success &&
            bundle.lbPoseResult.poseMatrix.valid && hasRaw &&
            bundle.mechEyeResult.pointCloud.isValid() &&
            bundle.mechEyeResult.pointCloud.pointsXYZ !=
                bundle.mechEyeResult.pointCloudRaw.pointsXYZ;

        // 约定：cloud.ply = 原始云；有 LB 时 cloud_stitched.ply = 变换云。
        const scan_tracking::mech_eye::PointCloudFrame& rawCloud =
            hasRaw ? bundle.mechEyeResult.pointCloudRaw : bundle.mechEyeResult.pointCloud;
        const QString plyPath = scan_tracking::mech_eye::buildSegmentPlyPath(
            runRoot, pathId, deviceLabel, segmentIndex);
        qInfo(LOG_SCAN_CACHE).noquote()
            << QStringLiteral("%1 段 %2 开始写 PLY cloud points=")
                   .arg(deviceLabel)
                   .arg(segmentIndex)
            << rawCloud.pointCount
            << QStringLiteral(" lbStitched=") << lbStitched;
        if (plyPath.isEmpty() ||
            !scan_tracking::mech_eye::savePointCloudFrameToPly(rawCloud, plyPath, texture)) {
            recordFailure(QStringLiteral("%1 段 %2 Mech-Eye 点云落盘失败")
                              .arg(deviceLabel)
                              .arg(segmentIndex));
        }

        if (lbStitched) {
            const QString stitchedPath = scan_tracking::mech_eye::buildSegmentStitchedPlyPath(
                runRoot, pathId, deviceLabel, segmentIndex);
            qInfo(LOG_SCAN_CACHE).noquote()
                << QStringLiteral("%1 段 %2 开始写 PLY cloud_stitched points=")
                       .arg(deviceLabel)
                       .arg(segmentIndex)
                << bundle.mechEyeResult.pointCloud.pointCount;
            // 拼接云落盘失败只记警告，不拖垮同段其它产物（避免再次因写盘问题连带整段落盘失败）。
            if (stitchedPath.isEmpty() ||
                !scan_tracking::mech_eye::savePointCloudFrameToPly(
                    bundle.mechEyeResult.pointCloud, stitchedPath, texture)) {
                qWarning(LOG_SCAN_CACHE).noquote()
                    << QStringLiteral("%1 段 %2 Mech-Eye cloud_stitched 落盘失败（已保留 cloud.ply）")
                           .arg(deviceLabel)
                           .arg(segmentIndex)
                    << QStringLiteral(" path=") << stitchedPath;
            }
        }
    }

    if (bundle.mechEyeResult.success()) {
        const bool textureValid = bundle.mechEyeResult.texture2D.isValid();
        qInfo(LOG_SCAN_CACHE).noquote()
            << QStringLiteral("%1 段 %2 开始写 Mech 纹理 PNG valid=")
                   .arg(deviceLabel)
                   .arg(segmentIndex)
            << textureValid
            << QStringLiteral(" size=")
            << bundle.mechEyeResult.texture2D.width << QLatin1Char('x')
            << bundle.mechEyeResult.texture2D.height;
        const QString pngPath = scan_tracking::mech_eye::buildSegmentMechTexturePngPath(
            runRoot, pathId, deviceLabel, segmentIndex);
        if (pngPath.isEmpty() ||
            !textureValid ||
            !scan_tracking::mech_eye::saveGrayTextureFrameToPng(
                bundle.mechEyeResult.texture2D, pngPath)) {
            recordFailure(QStringLiteral("%1 段 %2 Mech-Eye 纹理落盘失败")
                              .arg(deviceLabel)
                              .arg(segmentIndex));
        }
    }

    if (bundle.hikCameraCCaptureOk()) {
        const QString destPath = scan_tracking::vision::buildSegmentHikMonoPath(
            runRoot, pathId, deviceLabel, segmentIndex, QStringLiteral("hikC"));
        if (destPath.isEmpty()) {
            recordFailure(QStringLiteral("%1 段 %2 海康 C 落盘路径无效")
                              .arg(deviceLabel)
                              .arg(segmentIndex));
        } else if (!QFile::exists(bundle.hikCameraCImagePath)) {
            recordFailure(QStringLiteral("%1 段 %2 海康 C 源图不存在")
                              .arg(deviceLabel)
                              .arg(segmentIndex));
        } else {
            if (QFile::exists(destPath)) {
                QFile::remove(destPath);
            }
            if (!QFile::copy(bundle.hikCameraCImagePath, destPath)) {
                recordFailure(QStringLiteral("%1 段 %2 海康 C 落盘失败")
                                  .arg(deviceLabel)
                                  .arg(segmentIndex));
            }
        }
    } else if (bundle.hikCParticipated() && !bundle.hikCameraCTriggerOk && !cxpParticipated) {
        // 海康 C 仅 trigger-only（已发 start）时允许无本地图；真正采集失败才记错。
        recordFailure(QStringLiteral("%1 段 %2 海康 C 无有效帧")
                          .arg(deviceLabel)
                          .arg(segmentIndex));
    }

    if (cxpParticipated) {
        if (bundle.hikCameraAResult.frame.isValid()) {
            const QString bmpPath = scan_tracking::vision::buildSegmentHikMonoPath(
                runRoot, pathId, deviceLabel, segmentIndex, QStringLiteral("hikA"));
            if (bmpPath.isEmpty() ||
                !scan_tracking::vision::saveHikMonoFrameToBmp(
                    bundle.hikCameraAResult.frame, bmpPath)) {
                recordFailure(QStringLiteral("%1 段 %2 海康 A 落盘失败")
                                  .arg(deviceLabel)
                                  .arg(segmentIndex));
            }
        } else {
            recordFailure(QStringLiteral("%1 段 %2 海康 A 无有效帧")
                              .arg(deviceLabel)
                              .arg(segmentIndex));
        }

        if (bundle.hikCameraBResult.frame.isValid()) {
            const QString bmpPath = scan_tracking::vision::buildSegmentHikMonoPath(
                runRoot, pathId, deviceLabel, segmentIndex, QStringLiteral("hikB"));
            if (bmpPath.isEmpty() ||
                !scan_tracking::vision::saveHikMonoFrameToBmp(
                    bundle.hikCameraBResult.frame, bmpPath)) {
                recordFailure(QStringLiteral("%1 段 %2 海康 B 落盘失败")
                                  .arg(deviceLabel)
                                  .arg(segmentIndex));
            }
        } else {
            recordFailure(QStringLiteral("%1 段 %2 海康 B 无有效帧")
                              .arg(deviceLabel)
                              .arg(segmentIndex));
        }
    }

    // LB 位姿矩阵：与 Mech/CXP 同点位目录平铺（无 lb_pose 子目录）
    if (bundle.lbPoseResult.invoked) {
        QString lbError;
        if (!scan_tracking::vision::saveLbPoseResultToDisk(
                bundle.lbPoseResult,
                runRoot,
                pathId,
                deviceLabel,
                segmentIndex,
                &lbError)) {
            recordFailure(QStringLiteral("%1 段 %2 LB 位姿落盘失败：%3")
                              .arg(deviceLabel)
                              .arg(segmentIndex)
                              .arg(lbError));
        }
    }

    if (!ok) {
        if (errorMessage != nullptr) {
            *errorMessage = firstError;
        }
        qWarning(LOG_SCAN_CACHE).noquote()
            << QStringLiteral("%1 段 %2 落盘部分失败：")
                   .arg(deviceLabel)
                   .arg(segmentIndex)
            << firstError;
        return false;
    }

    qInfo(LOG_SCAN_CACHE).noquote()
        << QStringLiteral("%1 段 %2 已落盘至 %3")
               .arg(deviceLabel)
               .arg(segmentIndex)
               .arg(runRoot);
    return true;
}

void stripScanSegmentHeavyPayloads(vision::MultiCameraCaptureBundle* bundle)
{
    if (bundle == nullptr) {
        return;
    }

    mech_eye::CaptureResult& mech = bundle->mechEyeResult;

    // cloud.ply / cloud_stitched.ply 由落盘 job 持有的副本写出；检测仍读 pointCloud。
    mech.pointCloudRaw = mech_eye::PointCloudFrame{};

    if (mech.pointCloud.normalsXYZ) {
        mech.pointCloud.normalsXYZ.reset();
    }

    mech.texture2D = mech_eye::GrayTextureFrame{};

    bundle->hikCameraAResult.frame = vision::HikMonoFrame{};
    bundle->hikCameraBResult.frame = vision::HikMonoFrame{};
    bundle->hikCameraCImagePath.clear();
    bundle->heavyPayloadsStripped = true;
}

}  // namespace scan_tracking::flow_control
