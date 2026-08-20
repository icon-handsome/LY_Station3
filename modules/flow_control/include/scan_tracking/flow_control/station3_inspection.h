#pragma once

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"

#include <QtCore/QVector>

#include <memory>
#include <vector>

namespace scan_tracking::flow_control {

struct InspectionQuota {
    int expectedArmCount = 0;
    int expectedTelescopicCount = 0;
    int pathId = 0;
    QString pathName;
    QString algorithm;

    int total() const { return expectedArmCount + expectedTelescopicCount; }
};

struct InspectionSegmentCloud {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int localIndex = 0;
    bool captureOk = false;
    bool cxpParticipated = false;
    bool lbPoseOk = false;
    std::shared_ptr<std::vector<float>> xyz;
    int pointCount = 0;
};

struct InspectionCloudSnapshot {
    quint32 runTaskId = 0;
    QString runCaptureRoot;
    QVector<InspectionSegmentCloud> segments;

    void clear();
    int segmentCount() const { return segments.size(); }
    int countForDevice(common::ScanDeviceKind device) const;
    bool meetsDeviceQuotas(int expectedArmCount, int expectedTelescopicCount) const;
    const InspectionSegmentCloud* find(common::ScanDeviceKind device, int localIndex) const;
};

// Retained temporarily for the StateMachine's generic async extension points.
// S3 has no built-in incremental algorithm implementation.
struct IncrementalWeldSegmentResult {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int localIndex = 0;
    bool success = false;
    int errorCode = 0;
    QString errorMessage;
    double elapsedSeconds = 0.0;
};

bool prewarmActiveStation3InspectionAlgorithm(QString* errorMessage = nullptr);
InspectionCloudSnapshot buildInspectionCloudSnapshot(const ScanSegmentCache& cache);
bool buildInspectionSegmentCloud(
    const ScanSegmentCache& cache,
    common::ScanDeviceKind device,
    int localIndex,
    InspectionSegmentCloud* out);

IncrementalWeldSegmentResult evaluateWeldSectionSegment(
    const InspectionSegmentCloud& segment,
    int pathId,
    bool ringWeld);
InspectionResult aggregateWeldSectionSegments(
    const std::vector<IncrementalWeldSegmentResult>& segments,
    const InspectionQuota& quota,
    double wallElapsedSeconds);

InspectionResult evaluateStation3Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota);
InspectionResult evaluateStation3Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota);
bool tryEvaluateStation3Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out);
bool tryEvaluateStation3Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out);

}  // namespace scan_tracking::flow_control
