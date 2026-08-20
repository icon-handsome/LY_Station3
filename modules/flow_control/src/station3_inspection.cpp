#include "scan_tracking/flow_control/station3_inspection.h"

#include "scan_tracking/common/config_manager.h"

#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>

#include <algorithm>

Q_LOGGING_CATEGORY(LOG_STATION3_INSPECTION, "flow_control.station3_inspection")

namespace scan_tracking::flow_control {

namespace {

constexpr quint16 kNgReasonIncompleteSegments = 1u << 0;
constexpr quint16 kNgReasonBundleInvalid = 1u << 1;
constexpr quint16 kNgReasonPointCloudInvalid = 1u << 2;

InspectionQuota resolveQuota(InspectionQuota quota)
{
    const auto* config = common::ConfigManager::instance();
    if (config == nullptr) {
        return quota;
    }
    if (quota.pathId <= 0) {
        quota.pathId = config->activePathId();
    }
    if (quota.pathName.isEmpty()) {
        quota.pathName = config->activePathName();
    }
    if (quota.algorithm.isEmpty()) {
        quota.algorithm = config->activePathAlgorithm();
    }
    return quota;
}

void fillPathMeta(InspectionResult* result, const InspectionQuota& quota)
{
    result->pathId = quota.pathId;
    result->pathName = quota.pathName;
    result->algorithm = quota.algorithm;
}

InspectionResult evaluateCache(const ScanSegmentCache& cache, quint32 taskId, InspectionQuota quota)
{
    QElapsedTimer timer;
    timer.start();
    quota = resolveQuota(std::move(quota));

    InspectionResult result;
    fillPathMeta(&result, quota);
    result.sourcePointCount = cache.cachedSegmentCount();

    if (taskId != 0 && cache.runTaskId() != 0 && cache.runTaskId() != taskId) {
        result.resultCode = 3;
        result.message = QStringLiteral("S3 检测占位：任务号与当前扫描缓存不一致。");
    } else if (!cache.meetsDeviceQuotas(quota.expectedArmCount, quota.expectedTelescopicCount)) {
        result.resultCode = 3;
        result.ngReasonWord0 = kNgReasonIncompleteSegments;
        result.message = QStringLiteral("S3 检测占位：扫描段未齐套（机械臂 %1/%2，伸缩杆 %3/%4）。")
                             .arg(cache.cachedCountForDevice(common::ScanDeviceKind::Arm))
                             .arg(quota.expectedArmCount)
                             .arg(cache.cachedCountForDevice(common::ScanDeviceKind::Telescopic))
                             .arg(quota.expectedTelescopicCount);
    } else if (!cache.allCachedBundlesSuccessful()) {
        result.resultCode = 2;
        result.ngReasonWord0 = kNgReasonBundleInvalid;
        result.message = QStringLiteral("S3 检测占位：存在无效采集包。");
    } else {
        for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
            const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
            if (entry == nullptr || !entry->bundle.mechEyeResult.pointCloud.isValid()) {
                result.resultCode = 2;
                result.ngReasonWord0 = kNgReasonPointCloudInvalid;
                result.message = QStringLiteral("S3 检测占位：存在无效 Mech 点云。");
                result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
                return result;
            }
        }
        result.resultCode = 1;
        result.measurement.qualityCode = 1;
        result.message = QStringLiteral("S3 检测占位：采集完整性校验通过，等待接入算法 %1。")
                             .arg(quota.algorithm.isEmpty()
                                      ? QStringLiteral("<unconfigured>")
                                      : quota.algorithm);
    }

    result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
    return result;
}

}  // namespace

bool prewarmActiveStation3InspectionAlgorithm(QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    return true;
}

void InspectionCloudSnapshot::clear()
{
    runTaskId = 0;
    runCaptureRoot.clear();
    segments.clear();
}

int InspectionCloudSnapshot::countForDevice(common::ScanDeviceKind device) const
{
    return static_cast<int>(std::count_if(
        segments.cbegin(), segments.cend(),
        [device](const InspectionSegmentCloud& segment) { return segment.device == device; }));
}

bool InspectionCloudSnapshot::meetsDeviceQuotas(
    int expectedArmCount,
    int expectedTelescopicCount) const
{
    return (expectedArmCount <= 0 || countForDevice(common::ScanDeviceKind::Arm) >= expectedArmCount) &&
           (expectedTelescopicCount <= 0 ||
            countForDevice(common::ScanDeviceKind::Telescopic) >= expectedTelescopicCount) &&
           !segments.isEmpty();
}

const InspectionSegmentCloud* InspectionCloudSnapshot::find(
    common::ScanDeviceKind device,
    int localIndex) const
{
    for (const auto& segment : segments) {
        if (segment.device == device && segment.localIndex == localIndex) {
            return &segment;
        }
    }
    return nullptr;
}

bool buildInspectionSegmentCloud(
    const ScanSegmentCache& cache,
    common::ScanDeviceKind device,
    int localIndex,
    InspectionSegmentCloud* out)
{
    if (out == nullptr) {
        return false;
    }
    const ScanSegmentCacheEntry* entry = cache.entry(device, localIndex);
    if (entry == nullptr) {
        return false;
    }
    InspectionSegmentCloud segment;
    segment.device = device;
    segment.localIndex = localIndex;
    segment.captureOk = entry->bundle.success();
    segment.cxpParticipated = entry->bundle.cxpParticipated();
    const auto& pointCloud = entry->bundle.mechEyeResult.pointCloud;
    segment.xyz = pointCloud.pointsXYZ;
    segment.pointCount = pointCloud.pointCount;
    const auto& lbPose = entry->bundle.lbPoseResult;
    segment.lbPoseOk = lbPose.invoked && lbPose.success && lbPose.poseMatrix.valid;
    *out = std::move(segment);
    return true;
}

InspectionCloudSnapshot buildInspectionCloudSnapshot(const ScanSegmentCache& cache)
{
    InspectionCloudSnapshot snapshot;
    snapshot.runTaskId = cache.runTaskId();
    snapshot.runCaptureRoot = cache.runCaptureRoot();
    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        InspectionSegmentCloud segment;
        if (buildInspectionSegmentCloud(cache, key.device, key.localIndex, &segment)) {
            snapshot.segments.push_back(std::move(segment));
        }
    }
    return snapshot;
}

IncrementalWeldSegmentResult evaluateWeldSectionSegment(
    const InspectionSegmentCloud& segment,
    int pathId,
    bool ringWeld)
{
    Q_UNUSED(pathId);
    Q_UNUSED(ringWeld);
    IncrementalWeldSegmentResult result;
    result.device = segment.device;
    result.localIndex = segment.localIndex;
    result.errorCode = 2;
    result.errorMessage = QStringLiteral("S3 骨架未接入逐段算法。");
    return result;
}

InspectionResult aggregateWeldSectionSegments(
    const std::vector<IncrementalWeldSegmentResult>& segments,
    const InspectionQuota& quota,
    double wallElapsedSeconds)
{
    Q_UNUSED(segments);
    InspectionResult result;
    fillPathMeta(&result, resolveQuota(quota));
    result.resultCode = 3;
    result.elapsedSeconds = wallElapsedSeconds;
    result.message = QStringLiteral("S3 骨架未接入逐段算法。");
    return result;
}

InspectionResult evaluateStation3Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota)
{
    return evaluateCache(cache, taskId, quota);
}

InspectionResult evaluateStation3Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota)
{
    ScanSegmentCache cache;
    cache.bindExistingRunRoot(snapshot.runTaskId, snapshot.runCaptureRoot);
    for (const auto& segment : snapshot.segments) {
        vision::MultiCameraCaptureBundle bundle;
        bundle.mechEyeResult.errorCode = segment.captureOk
            ? mech_eye::CaptureErrorCode::Success
            : mech_eye::CaptureErrorCode::CaptureFailed;
        bundle.mechEyeResult.pointCloud.pointsXYZ = segment.xyz;
        bundle.mechEyeResult.pointCloud.pointCount = segment.pointCount;
        cache.storeSegment(segment.device, segment.localIndex, snapshot.runTaskId, std::move(bundle), false);
    }
    return evaluateCache(cache, taskId, quota);
}

bool tryEvaluateStation3Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out)
{
    if (out == nullptr) {
        return false;
    }
    *out = evaluateStation3Inspection(cache, taskId, quota);
    return true;
}

bool tryEvaluateStation3Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out)
{
    if (out == nullptr) {
        return false;
    }
    *out = evaluateStation3Inspection(snapshot, taskId, quota);
    return true;
}

}  // namespace scan_tracking::flow_control
