#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

void StateMachine::notifyScanStarted(int segmentIndex, quint32 taskId)
{
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        maybeEmitPathStarted(cfgMgr->activePathId());
    }
    emit scanStarted(segmentIndex, taskId);
}

void StateMachine::onBundleCaptureFinished(vision::MultiCameraCaptureBundle bundle)
{
    if (m_activeTask.definition == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：无活动任务 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::SelfCheck) {
        finishSelfCheckCapture(bundle);
        return;
    }
    if (!isScanCaptureStage(m_activeTask.definition->stage)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：当前阶段非扫描采集 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (m_activeTask.completionAnnounced) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：任务已收尾 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (bundle.request.requestId != m_activeTask.captureRequestId) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：requestId 不匹配 active=")
            << m_activeTask.captureRequestId
            << QStringLiteral(" bundle=") << bundle.request.requestId;
        return;
    }
    if (!acceptWorkpieceGeneration(
            m_activeTask.workpieceGeneration, QStringLiteral("bundleCaptureFinished"))) {
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("bundleCaptureFinished 处理 requestId=")
        << bundle.request.requestId
        << QStringLiteral(" segment=") << bundle.request.segmentIndex
        << QStringLiteral(" points=") << bundle.mechEyeResult.pointCloud.pointCount
        << QStringLiteral(" texture=") << bundle.mechEyeResult.texture2D.width
        << QLatin1Char('x') << bundle.mechEyeResult.texture2D.height
        << QStringLiteral(" textureValid=") << bundle.mechEyeResult.texture2D.isValid()
        << QStringLiteral(" lbInvoked=") << bundle.lbPoseResult.invoked;

    const QString triggerLabel = protocol::triggerName(*m_activeTask.definition);

    int imageCount = 0;
    int cloudFrameCount = 0;
    countBundleFrames(bundle, &imageCount, &cloudFrameCount);
    const QString bundleSummary = bundle.summary();
    const quint32 bundleTaskId = bundle.request.taskId;

    if (bundle.success()) {
        const auto device =
            m_activeTask.definition->stage == protocol::Stage::TelescopicScan
                ? common::ScanDeviceKind::Telescopic
                : common::ScanDeviceKind::Arm;

        const int segmentIndex = bundle.request.segmentIndex;

        // LB 点云拼接已在 VisionPipeline LB worker 完成；此处仅缓存与 ACK。
        m_scanSegmentCache.storeSegment(
            device,
            segmentIndex,
            bundleTaskId,
            std::move(bundle));

        // S3 骨架暂不绑定具体算法，采集结果保留在段缓存供 Inspection 使用。
        const bool incrementalWeldScheduled = false;

        const auto* configMgr = common::ConfigManager::instance();
        const int armExpected = configMgr != nullptr ? configMgr->enabledArmPointCount() : 0;
        const int telescopicExpected =
            configMgr != nullptr ? configMgr->enabledTelescopicPointCount() : 0;
        const int pathId = configMgr != nullptr ? configMgr->activePathId() : 0;
        qInfo(LOG_FLOW).noquote()
            << triggerLabel << QStringLiteral("：采集成功") << bundleSummary
            << QStringLiteral(" pathId=") << pathId
            << QStringLiteral(" imageCount=") << imageCount
            << QStringLiteral(" cloudFrameCount=") << cloudFrameCount
            << QStringLiteral(" cache arm=")
            << m_scanSegmentCache.cachedCountForDevice(common::ScanDeviceKind::Arm)
            << QStringLiteral("/") << armExpected
            << QStringLiteral(" telescopic=")
            << m_scanSegmentCache.cachedCountForDevice(common::ScanDeviceKind::Telescopic)
            << QStringLiteral("/") << telescopicExpected;

        const bool quotaComplete =
            m_scanSegmentCache.meetsDeviceQuotas(armExpected, telescopicExpected);
        if (quotaComplete) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("pathId=") << pathId
                << QStringLiteral(" 扫描齐套：将立即尝试后台解算；"
                                  "Trig_Inspection 到来时仅假成功放行。"
                                  "若 PLC 直接再发段号 1，IPC 将自动清缓存并切换到下一条启用路径。");
        }

        // 先回 PLC ACK，落盘在后台线程执行，避免阻塞 Modbus/HMI 事件循环。
        completeScanSegmentCapture(1, imageCount, cloudFrameCount, protocol::AckState::Completed, true);
        scheduleScanSegmentPersist(device, segmentIndex, triggerLabel);

        // 两个后台消费者均已通过 shared_ptr 接管。缓存不再长期持有主点云；
        // 缓冲会在落盘和 measureFrame 都完成后由最后一个持有者释放。
        if (incrementalWeldScheduled &&
            !m_scanSegmentCache.releaseSegmentPointCloud(device, segmentIndex)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("释放逐段焊缝点云失败，段不在缓存 device=")
                << common::ConfigManager::scanDeviceKindToString(device)
                << QStringLiteral(" segment=") << segmentIndex;
        }

        // 利用机械臂/伸缩杆回位真空期：齐套后立刻抽快照并后台解算。
        if (quotaComplete) {
            maybeStartInspectionSolveWhenQuotaComplete(QStringLiteral("QuotaComplete"));
        }
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << triggerLabel << QStringLiteral("：采集失败") << bundle.summary();
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::onVisionPipelineFatalError(vision::VisionErrorCode code, QString message)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[VisionPipeline] 致命错误：")
        << static_cast<int>(code)
        << message;

    if (m_activeTask.definition == nullptr || m_activeTask.completionAnnounced) {
        return;
    }

    if (m_activeTask.definition->stage == protocol::Stage::SelfCheck) {
        setAlarm(3, 723, message);
        constexpr quint16 kFailCapture = 1u << 4;
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        return;
    }

    if (!isScanCaptureStage(m_activeTask.definition->stage)) {
        return;
    }

    setAlarm(3, 723, message);
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::completeScanSegmentCapture(
    quint16 resultCode,
    int imageCount,
    int cloudFrameCount,
    protocol::AckState finalAckState,
    bool dataValid)
{
    const int segmentIndex = m_activeTask.scanSegmentIndex;
    const auto device = activeScanDeviceKind();
    const int pathId =
        common::ConfigManager::instance() != nullptr
            ? common::ConfigManager::instance()->activePathId()
            : 0;

    // 对齐工位1：严重错误（Res>=5）按 scanFailurePolicy 清理，默认不整表抹掉已扫段。
    if (resultCode >= 5) {
        applyScanFailurePolicy(pathId, device, segmentIndex, resultCode);
    }

    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::TelescopicScan) {
        writeTelescopicScanResult(segmentIndex, imageCount, cloudFrameCount);
    } else {
        writeScanSegmentResult(segmentIndex, imageCount, cloudFrameCount);
    }
    completeActiveTask(resultCode, finalAckState, dataValid);
    emit scanFinished(segmentIndex, resultCode, imageCount, cloudFrameCount);
}

common::ScanDeviceKind StateMachine::activeScanDeviceKind() const
{
    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::TelescopicScan) {
        return common::ScanDeviceKind::Telescopic;
    }
    return common::ScanDeviceKind::Arm;
}

QString StateMachine::currentScanFailurePolicy() const
{
    if (const auto* cfg = common::ConfigManager::instance()) {
        const QString policy = cfg->flowControlConfig().scanFailurePolicy.trimmed().toLower();
        if (policy == QLatin1String("path") || policy == QLatin1String("workpiece")) {
            return policy;
        }
    }
    return QStringLiteral("segment");
}

void StateMachine::applyScanFailurePolicy(
    int pathId,
    common::ScanDeviceKind device,
    int segmentIndex,
    quint16 resultCode)
{
    const QString policy = currentScanFailurePolicy();

    if (policy == QLatin1String("workpiece")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] 策略=workpiece，整表清缓存 Res=") << resultCode
            << QStringLiteral(" pathId=") << pathId;
        bumpWorkpieceGeneration(QStringLiteral("scan_fail_workpiece"));
        clearTransientWorkpieceRuntimeState();
        resetScanSegmentCache();
        resetActivePathToFirstEnabled();
        clearPathProgressTracking(QStringLiteral("scan_fail_workpiece"));
        return;
    }

    if (policy == QLatin1String("path")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] 策略=path，清当前路径段缓存 Res=") << resultCode
            << QStringLiteral(" pathId=") << pathId;
        clearScanSegmentCacheForPathSwitch();
        m_lastInspectedPathId = -1;
        m_lastInspectedRunKey.clear();
        if (pathId > 0) {
            m_emittedPathFinished.remove(pathId);
        }
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[ScanFail] 策略=segment，仅剔失败本段 Res=") << resultCode
        << QStringLiteral(" pathId=") << pathId
        << QStringLiteral(" device=")
        << common::ConfigManager::scanDeviceKindToString(device)
        << QStringLiteral(" 段号=") << segmentIndex;
    if (segmentIndex > 0 && m_scanSegmentCache.removeSegment(device, segmentIndex)) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] 已从段缓存移除失败段");
    }
    // 失败段可能曾被标为已检测：允许同路径重试检测。
    if (m_lastInspectedPathId == pathId) {
        m_lastInspectedPathId = -1;
        m_lastInspectedRunKey.clear();
    }
}

void StateMachine::applyInspectionTimeoutFailurePolicy()
{
    // 对齐工位1：检测超时先作废在途后台结果，再按策略清缓存。
    bumpWorkpieceGeneration(QStringLiteral("inspection_timeout"));

    const QString policy = currentScanFailurePolicy();
    const int pathId =
        common::ConfigManager::instance() != nullptr
            ? common::ConfigManager::instance()->activePathId()
            : 0;

    // 超时不得视为「已检测完成」，否则会挡住同路径重试。
    m_lastInspectedPathId = -1;
    m_lastInspectedRunKey.clear();

    if (policy == QLatin1String("workpiece")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] Inspection 超时策略=workpiece，整表清缓存");
        clearTransientWorkpieceRuntimeState();
        resetScanSegmentCache();
        resetActivePathToFirstEnabled();
        clearPathProgressTracking(QStringLiteral("inspection_timeout_workpiece"));
        return;
    }

    if (policy == QLatin1String("path")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] Inspection 超时策略=path，清当前路径段缓存 pathId=")
            << pathId;
        clearScanSegmentCacheForPathSwitch();
        if (pathId > 0) {
            m_emittedPathFinished.remove(pathId);
        }
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[ScanFail] Inspection 超时策略=segment，保留段缓存供重试 pathId=")
        << pathId;
}

void StateMachine::onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message)
{
    Q_UNUSED(code);
    qCritical(LOG_FLOW) << "[MechEye] 致命错误:" << message;
    emit protocolEvent(QStringLiteral("Mech-Eye: %1").arg(message));

    if (m_activeTask.definition == nullptr ||
        !isScanCaptureStage(m_activeTask.definition->stage) ||
        m_activeTask.completionAnnounced) {
        return;
    }

    setAlarm(3, 723, message);
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::resetScanSegmentCache()
{
    resetIncrementalWeldState();
    m_latestScanPersistBarrier = {};
    m_scanSegmentCache.reset();
    qInfo(LOG_FLOW).noquote() << QStringLiteral("扫描段缓存已清空（含运行实例目录绑定）。");
}

void StateMachine::scheduleScanSegmentPersist(
    common::ScanDeviceKind device,
    int segmentIndex,
    const QString& triggerLabel)
{
    const ScanSegmentCacheEntry* entry = m_scanSegmentCache.entry(device, segmentIndex);
    if (entry == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << triggerLabel << QStringLiteral("：后台落盘跳过，段不在缓存中 segment=")
            << segmentIndex;
        return;
    }

    ScanSegmentPersistJob job;
    job.runRoot = entry->runCaptureRoot;
    job.device = entry->device;
    job.segmentIndex = entry->segmentIndex;
    job.taskId = entry->taskId;
    job.captureTimestamp = entry->captureTimestamp;
    job.bundle = entry->bundle;
    job.triggerLabel = triggerLabel;

    qInfo(LOG_FLOW).noquote()
        << triggerLabel << QStringLiteral("：已投递后台落盘 taskId=") << job.taskId
        << QStringLiteral(" runRoot=") << job.runRoot
        << QStringLiteral(" segment=") << segmentIndex;

    m_latestScanPersistBarrier = m_scanPersistWorker.enqueue(std::move(job));

    // 落盘 job 已持有 shared_ptr 副本；立即从缓存剥离纹理/raw/CXP，避免等磁盘期间双份驻留。
    if (!m_scanSegmentCache.stripHeavyPayloads(device, segmentIndex)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("投递落盘后剥离重载荷失败，段不在缓存 segment=")
            << segmentIndex;
    }
}

void StateMachine::onScanSegmentPersistFinished(
    common::ScanDeviceKind device,
    int segmentIndex,
    bool ok)
{
    Q_UNUSED(device);
    // 重载荷已在 enqueue 后剥离；此处仅保留回调钩子供日志/后续扩展。
    if (!ok) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台落盘失败（缓存重载荷已剥离，检测点云仍保留） segment=")
            << segmentIndex;
    }
}

void StateMachine::clearScanSegmentCacheForPathSwitch()
{
    resetIncrementalWeldState();
    m_latestScanPersistBarrier = {};
    m_scanSegmentCache.clearSegmentsKeepRunRoot();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已清段缓存并保留运行实例目录：")
        << m_scanSegmentCache.runCaptureRoot();
}

}  // namespace scan_tracking::flow_control
