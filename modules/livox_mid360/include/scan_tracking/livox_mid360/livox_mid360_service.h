#pragma once

#include <QtCore/QObject>

#include "scan_tracking/livox_mid360/livox_mid360_types.h"

QT_BEGIN_NAMESPACE
class QThread;
QT_END_NAMESPACE

namespace scan_tracking {
namespace livox_mid360 {

class LivoxMid360Worker;

class LivoxMid360Service : public QObject {
    Q_OBJECT

public:
    explicit LivoxMid360Service(QObject* parent = nullptr);
    ~LivoxMid360Service() override;

    void start();
    void stop();

    LivoxMid360RuntimeState state() const { return m_currentState; }

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

private slots:
    void onWorkerEnumerateFinished(
        QVector<scan_tracking::livox_mid360::LivoxMid360DeviceSummary> devices);
    void onWorkerOpenFinished(
        bool success,
        scan_tracking::livox_mid360::LivoxMid360DeviceSummary deviceInfo,
        QString errorMessage);
    void onWorkerStateChanged(
        scan_tracking::livox_mid360::LivoxMid360RuntimeState newState,
        QString description);
    void onWorkerLogMessage(QString message);

signals:
    void sig_startWorker(scan_tracking::livox_mid360::LivoxMid360OpenConfig config);
    void sig_stopWorker();

private:
    static void registerMetaTypes();

    QThread* m_workerThread = nullptr;
    LivoxMid360Worker* m_worker = nullptr;
    LivoxMid360OpenConfig m_openConfig;
    LivoxMid360RuntimeState m_currentState = LivoxMid360RuntimeState::Idle;
    bool m_started = false;
    bool m_stopping = false;
};

}  // namespace livox_mid360
}  // namespace scan_tracking
