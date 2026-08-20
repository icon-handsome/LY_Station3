#pragma once

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QVector>

namespace scan_tracking {
namespace livox_mid360 {

enum class LivoxMid360RuntimeState {
    Idle = 0,
    Enumerating,
    Opening,
    Ready,
    Failed,
    Stopped,
};

struct LivoxMid360OpenConfig {
    QString configFilePath;
    QString serial;
    int discoveryTimeoutMs = 10000;
};

struct LivoxMid360DeviceSummary {
    int index = -1;
    quint32 handle = 0;
    QString serialNumber;
    QString lidarIp;
    QString handleIp;
    int deviceType = -1;
    QString deviceTypeName;
};

}  // namespace livox_mid360
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::livox_mid360::LivoxMid360RuntimeState)
Q_DECLARE_METATYPE(scan_tracking::livox_mid360::LivoxMid360OpenConfig)
Q_DECLARE_METATYPE(scan_tracking::livox_mid360::LivoxMid360DeviceSummary)
Q_DECLARE_METATYPE(QVector<scan_tracking::livox_mid360::LivoxMid360DeviceSummary>)
