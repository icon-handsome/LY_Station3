#include "scan_tracking/tfmini_plus/tfmini_plus_worker.h"

#include <QtCore/QLoggingCategory>
#include <QtSerialPort/QSerialPort>

Q_LOGGING_CATEGORY(LOG_TFMINI_PLUS, "tfmini.plus")
Q_LOGGING_CATEGORY(LOG_TFMINI_PLUS_FRAME, "tfmini.plus.frame")

namespace scan_tracking {
namespace tfmini_plus {

namespace {

int byteAt(const QByteArray& data, int index)
{
    return static_cast<unsigned char>(data.at(index));
}

bool hasValidChecksum(const QByteArray& frame)
{
    int checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum = (checksum + byteAt(frame, i)) & 0xFF;
    }
    return checksum == byteAt(frame, 8);
}

constexpr int kFrameSize = 9;
constexpr char kFrameHeader = '\x59';
constexpr int kMaxRxBufferBytes = 512;
constexpr int kStrengthUnreliableThreshold = 100;
constexpr int kStrengthOverexposed = 65535;

void trimRxBuffer(QByteArray& buffer)
{
    if (buffer.size() > kMaxRxBufferBytes) {
        buffer.remove(0, buffer.size() - kMaxRxBufferBytes);
    }
}

bool tryParseFrame(const QByteArray& frame, TfminiPlusFrame* out)
{
    if (out == nullptr || frame.size() < kFrameSize) {
        return false;
    }
    if (byteAt(frame, 0) != kFrameHeader || byteAt(frame, 1) != kFrameHeader) {
        return false;
    }

    out->distanceCm = byteAt(frame, 2) | (byteAt(frame, 3) << 8);
    out->strength = byteAt(frame, 4) | (byteAt(frame, 5) << 8);
    // Byte6/7 为芯片温度，按需求不解析。
    out->checksumValid = hasValidChecksum(frame);
    if (!out->checksumValid) {
        return false;
    }

    out->isReliable =
        out->strength >= kStrengthUnreliableThreshold
        && out->strength != kStrengthOverexposed;
    return true;
}

}  // namespace

TfminiPlusWorker::TfminiPlusWorker(QObject* parent)
    : QObject(parent)
{
}

TfminiPlusWorker::~TfminiPlusWorker()
{
    stopWorker();
}

QString TfminiPlusWorker::logPrefix() const
{
    if (m_deviceLabel.trimmed().isEmpty()) {
        return QStringLiteral("[TfminiPlus]");
    }
    return QStringLiteral("[%1]").arg(m_deviceLabel.trimmed());
}

void TfminiPlusWorker::startWorker(const TfminiPlusOpenConfig& config)
{
    stopWorker();

    m_buffer.clear();
    m_logFrames = config.logFrames;
    m_deviceLabel = config.deviceLabel.trimmed();

    const QString portName = config.portName.trimmed();
    if (portName.isEmpty()) {
        const QString message = QStringLiteral("Serial port is empty; set tfminiPlusPort / tfminiPlusPort2 in [TfminiPlus]");
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        emit openFinished(false, message);
        emit stateChanged(TfminiPlusRuntimeState::Failed, message);
        return;
    }

    emit stateChanged(TfminiPlusRuntimeState::Opening, QStringLiteral("Opening serial port"));

    m_serialPort = new QSerialPort(this);
    m_serialPort->setPortName(portName);
    m_serialPort->setBaudRate(config.baudRate > 0 ? config.baudRate : 115200);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    connect(m_serialPort, &QSerialPort::readyRead, this, &TfminiPlusWorker::onReadyRead);
    connect(
        m_serialPort,
        QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
        this,
        &TfminiPlusWorker::onSerialError);

    if (!m_serialPort->open(QIODevice::ReadOnly)) {
        const QString message = QStringLiteral("Open serial port failed: %1 (%2)")
                                    .arg(portName, m_serialPort->errorString());
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        emit openFinished(false, message);
        emit stateChanged(TfminiPlusRuntimeState::Failed, message);
        teardownSerial();
        return;
    }

    const QString openedMessage = QStringLiteral("Opened %1 baud=%2 8N1")
                                      .arg(portName)
                                      .arg(m_serialPort->baudRate());
    emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), openedMessage));
    emit openFinished(true, {});
    emit stateChanged(TfminiPlusRuntimeState::Ready, openedMessage);
}

void TfminiPlusWorker::stopWorker()
{
    const bool wasOpen = m_serialPort != nullptr;
    teardownSerial();
    m_buffer.clear();
    if (wasOpen) {
        emit stateChanged(TfminiPlusRuntimeState::Stopped, QStringLiteral("Worker stopped"));
    }
}

void TfminiPlusWorker::onReadyRead()
{
    if (m_serialPort == nullptr) {
        return;
    }

    const QByteArray data = m_serialPort->readAll();
    if (data.isEmpty()) {
        return;
    }

    m_buffer.append(data);
    trimRxBuffer(m_buffer);
    parseBuffer();
}

void TfminiPlusWorker::onSerialError()
{
    if (m_serialPort == nullptr) {
        return;
    }

    const auto error = m_serialPort->error();
    if (error == QSerialPort::NoError || error == QSerialPort::TimeoutError) {
        return;
    }

    const QString message = QStringLiteral("Serial port error: %1").arg(m_serialPort->errorString());
    emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
    emit stateChanged(TfminiPlusRuntimeState::Failed, message);
}

void TfminiPlusWorker::parseBuffer()
{
    while (m_buffer.size() >= 2) {
        int headerIndex = -1;
        for (int i = 0; i + 1 < m_buffer.size(); ++i) {
            if (static_cast<unsigned char>(m_buffer.at(i)) == kFrameHeader
                && static_cast<unsigned char>(m_buffer.at(i + 1)) == kFrameHeader) {
                headerIndex = i;
                break;
            }
        }

        if (headerIndex < 0) {
            m_buffer.clear();
            return;
        }
        if (headerIndex > 0) {
            m_buffer.remove(0, headerIndex);
        }
        if (m_buffer.size() < kFrameSize) {
            return;
        }

        const QByteArray rawFrame = m_buffer.left(kFrameSize);
        TfminiPlusFrame frame;
        if (!tryParseFrame(rawFrame, &frame)) {
            m_buffer.remove(0, 1);
            continue;
        }

        if (m_logFrames) {
            qInfo(LOG_TFMINI_PLUS_FRAME).noquote()
                << QStringLiteral("%1 dist=%2cm strength=%3 reliable=%4")
                       .arg(logPrefix())
                       .arg(frame.distanceCm)
                       .arg(frame.strength)
                       .arg(frame.isReliable ? QStringLiteral("yes")
                                             : QStringLiteral("no"));
        }
        emit distanceUpdated(frame.distanceCm, frame.strength);
        m_buffer.remove(0, kFrameSize);
    }
}

void TfminiPlusWorker::teardownSerial()
{
    if (m_serialPort == nullptr) {
        return;
    }

    if (m_serialPort->isOpen()) {
        m_serialPort->close();
    }
    delete m_serialPort;
    m_serialPort = nullptr;
}

}  // namespace tfmini_plus
}  // namespace scan_tracking
