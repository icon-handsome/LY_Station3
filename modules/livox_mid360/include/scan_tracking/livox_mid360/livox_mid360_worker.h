#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/livox_mid360/livox_mid360_types.h"

namespace scan_tracking {
namespace livox_mid360 {

class LivoxMid360Worker : public QObject {
    Q_OBJECT

public:
    explicit LivoxMid360Worker(QObject* parent = nullptr);
    ~LivoxMid360Worker() override;

public slots:
    void startWorker(const scan_tracking::livox_mid360::LivoxMid360OpenConfig& config);
    void stopWorker();
    void onDeviceDiscovered(
        quint32 handle,
        QString serialNumber,
        QString lidarIp,
        int deviceType);
    void onDiscoveryTimeout();

signals:
    void enumerateFinished(QVector<scan_tracking::livox_mid360::LivoxMid360DeviceSummary> devices);
    void openFinished(
        bool success,
        scan_tracking::livox_mid360::LivoxMid360DeviceSummary deviceInfo,
        QString errorMessage);
    void stateChanged(
        scan_tracking::livox_mid360::LivoxMid360RuntimeState newState,
        QString description);
    void logMessage(QString message);
    void discoveryWindowFinished();

private:
    void finishDiscovery();
    void teardownSdk();
    void cleanupTempConfigFile();
    QString resolveConfigPathForSdk(const QString& configPath, QString* warningMessage);

    bool m_sdkInitialized = false;
    bool m_discoveryActive = false;
    int m_nextDeviceIndex = 0;
    QVector<LivoxMid360DeviceSummary> m_discoveredDevices;
    LivoxMid360OpenConfig m_openConfig;
    QString m_tempConfigPath;
};

}  // namespace livox_mid360
}  // namespace scan_tracking
