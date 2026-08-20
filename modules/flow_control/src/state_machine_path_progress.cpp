#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

namespace {

bool isHmiTrackedPath(int pathId, const common::ScanPathConfig* path)
{
    if (pathId <= 0 || path == nullptr) {
        return false;
    }
    const QString algorithm = common::ConfigManager::resolvePathAlgorithm(*path).trimmed();
    return algorithm != QLatin1String("self_check");
}

}  // namespace

ScanPathEventInfo StateMachine::buildScanPathEventInfo(int pathId, quint16 resultCode) const
{
    ScanPathEventInfo info;
    info.pathId = pathId;
    info.resultCode = resultCode;

    const auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        info.pathName = QStringLiteral("路径%1").arg(pathId);
        return info;
    }

    const QVector<int> pathIds = cfgMgr->enabledPathIds();
    info.pathCount = pathIds.size();
    const int index = pathIds.indexOf(pathId);
    info.pathIndex = index >= 0 ? index + 1 : 0;

    if (const common::ScanPathConfig* path = cfgMgr->findScanPathById(pathId)) {
        info.pathName = path->name.isEmpty() ? QStringLiteral("路径%1").arg(pathId) : path->name;
        info.inspectionType = common::ConfigManager::resolvePathAlgorithm(*path);
        info.totalPoints = path->totalPoints > 0
            ? path->totalPoints
            : (path->armPointCount + path->telescopicPointCount);
    } else {
        info.pathName = QStringLiteral("路径%1").arg(pathId);
    }
    return info;
}

void StateMachine::maybeEmitPathStarted(int pathId)
{
    const auto* cfgMgr = common::ConfigManager::instance();
    const common::ScanPathConfig* path =
        cfgMgr != nullptr ? cfgMgr->findScanPathById(pathId) : nullptr;
    if (!isHmiTrackedPath(pathId, path)) {
        return;
    }
    if (m_emittedPathStarted.contains(pathId)) {
        return;
    }

    m_emittedPathStarted.insert(pathId);
    const ScanPathEventInfo info = buildScanPathEventInfo(pathId);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[HMI路径] 开始 pathId=") << info.pathId
        << QStringLiteral(" name=") << info.pathName
        << QStringLiteral(" algorithm=") << info.inspectionType
        << QStringLiteral(" index=") << info.pathIndex << QLatin1Char('/') << info.pathCount;
    emit pathStarted(info);
}

void StateMachine::maybeEmitPathFinished(int pathId, quint16 resultCode)
{
    const auto* cfgMgr = common::ConfigManager::instance();
    const common::ScanPathConfig* path =
        cfgMgr != nullptr ? cfgMgr->findScanPathById(pathId) : nullptr;
    if (!isHmiTrackedPath(pathId, path)) {
        return;
    }
    if (m_emittedPathFinished.contains(pathId)) {
        return;
    }

    m_emittedPathFinished.insert(pathId);
    m_emittedPathStarted.insert(pathId);  // 完成即视为已开始过
    const ScanPathEventInfo info = buildScanPathEventInfo(pathId, resultCode);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[HMI路径] 完成 pathId=") << info.pathId
        << QStringLiteral(" name=") << info.pathName
        << QStringLiteral(" index=") << info.pathIndex << QLatin1Char('/') << info.pathCount
        << QStringLiteral(" resultCode=") << resultCode;
    emit pathFinished(info);

    if (cfgMgr == nullptr || m_emittedAllPathsFinished) {
        return;
    }

    const QVector<int> pathIds = cfgMgr->enabledPathIds();
    QVector<int> completedPathIds;
    completedPathIds.reserve(pathIds.size());
    for (int id : pathIds) {
        const common::ScanPathConfig* enabledPath = cfgMgr->findScanPathById(id);
        if (!isHmiTrackedPath(id, enabledPath)) {
            continue;
        }
        if (!m_emittedPathFinished.contains(id)) {
            return;
        }
        completedPathIds.append(id);
    }

    if (completedPathIds.isEmpty()) {
        return;
    }

    m_emittedAllPathsFinished = true;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[HMI路径] 全部启用路径完成，共")
        << completedPathIds.size() << QStringLiteral("条");
    emit scanPathsAllFinished(completedPathIds, completedPathIds.size());
}

void StateMachine::clearPathProgressTracking(const QString& resetReason)
{
    m_emittedPathStarted.clear();
    m_emittedPathFinished.clear();
    m_emittedAllPathsFinished = false;
    if (!resetReason.isEmpty()) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[HMI路径] 进度复位 reason=") << resetReason;
        emit pathProgressReset(resetReason);
    }
}

ScanPathProgressSnapshot StateMachine::scanPathProgressSnapshot() const
{
    ScanPathProgressSnapshot snapshot;
    const auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return snapshot;
    }

    for (int pathId : cfgMgr->enabledPathIds()) {
        const common::ScanPathConfig* path = cfgMgr->findScanPathById(pathId);
        if (!isHmiTrackedPath(pathId, path)) {
            continue;
        }
        snapshot.enabledPathIds.append(pathId);
        if (m_emittedPathFinished.contains(pathId)) {
            snapshot.completedPathIds.append(pathId);
        }
    }
    snapshot.pathCount = snapshot.enabledPathIds.size();
    snapshot.allPathsComplete =
        snapshot.pathCount > 0 &&
        snapshot.completedPathIds.size() == snapshot.pathCount;

    if (snapshot.allPathsComplete) {
        snapshot.currentPathId = 0;
        snapshot.currentPathName.clear();
        return snapshot;
    }

    const int activeId = cfgMgr->activePathId();
    const common::ScanPathConfig* activePath = cfgMgr->findScanPathById(activeId);
    if (isHmiTrackedPath(activeId, activePath)) {
        snapshot.currentPathId = activeId;
        snapshot.currentPathName = cfgMgr->activePathName();
    } else if (!snapshot.enabledPathIds.isEmpty()) {
        snapshot.currentPathId = snapshot.enabledPathIds.front();
        if (const common::ScanPathConfig* fallback =
                cfgMgr->findScanPathById(snapshot.currentPathId)) {
            snapshot.currentPathName = fallback->name;
        }
    }
    return snapshot;
}

}  // namespace scan_tracking::flow_control
