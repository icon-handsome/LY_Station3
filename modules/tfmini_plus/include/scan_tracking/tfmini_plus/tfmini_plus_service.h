#pragma once

#include <QtCore/QObject>

#include "scan_tracking/tfmini_plus/tfmini_plus_types.h"

QT_BEGIN_NAMESPACE
class QThread;
QT_END_NAMESPACE

namespace scan_tracking {
namespace tfmini_plus {

class TfminiPlusWorker;

class TfminiPlusService : public QObject {
    Q_OBJECT

public:
    explicit TfminiPlusService(QObject* parent = nullptr);
    ~TfminiPlusService() override;

    /// 启动前设置串口参数；未调用时 start() 从 ConfigManager 读取单口配置。
    void configure(const TfminiPlusOpenConfig& config);

    void start();
    void stop();

    TfminiPlusRuntimeState state() const { return m_currentState; }
    QString deviceLabel() const { return m_openConfig.deviceLabel; }

signals:
    void openFinished(bool success, QString errorMessage);
    void stateChanged(
        scan_tracking::tfmini_plus::TfminiPlusRuntimeState newState,
        QString description);
    void distanceUpdated(int distanceCm, int strength);
    void logMessage(QString message);

signals:
    void sig_startWorker(scan_tracking::tfmini_plus::TfminiPlusOpenConfig config);
    void sig_stopWorker();

private slots:
    void onWorkerOpenFinished(bool success, QString errorMessage);
    void onWorkerStateChanged(
        scan_tracking::tfmini_plus::TfminiPlusRuntimeState newState,
        QString description);
    void onWorkerDistanceUpdated(int distanceCm, int strength);
    void onWorkerLogMessage(QString message);

private:
    static void registerMetaTypes();

    QThread* m_workerThread = nullptr;
    TfminiPlusWorker* m_worker = nullptr;
    TfminiPlusOpenConfig m_openConfig;
    bool m_openConfigSet = false;
    TfminiPlusRuntimeState m_currentState = TfminiPlusRuntimeState::Idle;
    bool m_started = false;
};

}  // namespace tfmini_plus
}  // namespace scan_tracking
