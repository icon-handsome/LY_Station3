#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QObject>
#include <QtCore/QString>

#include "scan_tracking/tfmini_plus/tfmini_plus_types.h"

QT_BEGIN_NAMESPACE
class QSerialPort;
QT_END_NAMESPACE

namespace scan_tracking {
namespace tfmini_plus {

class TfminiPlusWorker : public QObject {
    Q_OBJECT

public:
    explicit TfminiPlusWorker(QObject* parent = nullptr);
    ~TfminiPlusWorker() override;

public slots:
    void startWorker(const scan_tracking::tfmini_plus::TfminiPlusOpenConfig& config);
    void stopWorker();

signals:
    void openFinished(bool success, QString errorMessage);
    void stateChanged(
        scan_tracking::tfmini_plus::TfminiPlusRuntimeState newState,
        QString description);
    void distanceUpdated(int distanceCm, int strength);
    void logMessage(QString message);

private slots:
    void onReadyRead();
    void onSerialError();

private:
    void parseBuffer();
    void teardownSerial();

    QString logPrefix() const;

    QSerialPort* m_serialPort = nullptr;
    QByteArray m_buffer;
    bool m_logFrames = false;
    QString m_deviceLabel;
};

}  // namespace tfmini_plus
}  // namespace scan_tracking
