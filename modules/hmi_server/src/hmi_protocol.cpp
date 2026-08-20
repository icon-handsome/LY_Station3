/**
 * @file hmi_protocol.cpp
 * @brief HMI 显控通信协议 JSON 组包辅助函数实现
 */

#include "scan_tracking/hmi_server/hmi_protocol.h"

#include <QtCore/QDateTime>
#include <QtCore/QJsonDocument>
#include <QtEndian>

namespace scan_tracking {
namespace hmi_server {

QJsonObject buildEnvelope(const QString& type, const QString& msgId, const QJsonObject& payload)
{
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = type;
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    return envelope;
}

QJsonObject buildResponsePayload(bool success, const QString& message)
{
    QJsonObject payload;
    payload[QStringLiteral("success")] = success;
    payload[QStringLiteral("message")] = message;
    return payload;
}

QByteArray serializeFrame(const QJsonObject& envelope)
{
    // 将 JSON 对象序列化为紧凑格式的 UTF-8 字节流
    const QByteArray json = QJsonDocument(envelope).toJson(QJsonDocument::Compact);
    const quint32 length = static_cast<quint32>(json.size());

    // 构建帧：4 字节大端长度头 + JSON 正文
    QByteArray frame(4, 0);
    qToBigEndian(length, reinterpret_cast<uchar*>(frame.data()));
    frame.append(json);
    return frame;
}

QString frameJsonToLogString(const QByteArray& jsonUtf8)
{
    return QString::fromUtf8(jsonUtf8);
}

QString summarizeHmiTracePayload(const QString& type, const QJsonObject& payload)
{
    using namespace msg_type;

    if (type == QLatin1String(kStatusSystem)) {
        return QStringLiteral("ipcState=%1 appState=%2 alarmLevel=%3 progress=%4")
            .arg(payload.value(QLatin1String("ipcState")).toInt(-1))
            .arg(payload.value(QLatin1String("appState")).toString(),
                 QString::number(payload.value(QLatin1String("alarmLevel")).toInt(-1)),
                 QString::number(payload.value(QLatin1String("progress")).toInt(-1)));
    }
    if (type == QLatin1String(kStatusPlc)) {
        QString summary = QStringLiteral("modbusConnected=%1")
                              .arg(payload.value(QLatin1String("modbusConnected")).toBool() ? 1 : 0);
        if (payload.contains(QLatin1String("workMode"))) {
            summary += QStringLiteral(" workMode=%1").arg(payload.value(QLatin1String("workMode")).toInt());
        }
        if (payload.contains(QLatin1String("scanSegmentIndex"))) {
            summary += QStringLiteral(" seg=%1/%2")
                            .arg(payload.value(QLatin1String("scanSegmentIndex")).toInt())
                            .arg(payload.value(QLatin1String("scanSegmentTotal")).toInt());
        }
        if (payload.contains(QLatin1String("robotStatusWord"))) {
            summary += QStringLiteral(" robot=0x%1")
                            .arg(payload.value(QLatin1String("robotStatusWord")).toInt(0), 4, 16, QLatin1Char('0'));
        }
        if (payload.contains(QLatin1String("telescopicRodStatus"))) {
            summary += QStringLiteral(" rod=%1 mag=%2 estop=%3")
                            .arg(payload.value(QLatin1String("telescopicRodStatus")).toInt())
                            .arg(payload.value(QLatin1String("electromagnetStatus")).toInt())
                            .arg(payload.value(QLatin1String("estopButtonStatus")).toInt());
        }
        return summary;
    }
    if (type == QLatin1String(kStatusCamera)) {
        QStringList parts;
        const QJsonObject mechEye = payload.value(QLatin1String("mechEye")).toObject();
        if (!mechEye.isEmpty()) {
            parts << QStringLiteral("梅卡(st=%1,conn=%2)")
                         .arg(mechEye.value(QLatin1String("state")).toInt(-1))
                         .arg(mechEye.value(QLatin1String("connected")).toBool() ? 1 : 0);
        }
        const QJsonObject hikA = payload.value(QLatin1String("hikA")).toObject();
        if (!hikA.isEmpty()) {
            parts << QStringLiteral("hikA(conn=%1)")
                         .arg(hikA.value(QLatin1String("connected")).toBool() ? 1 : 0);
        }
        const QJsonObject hikB = payload.value(QLatin1String("hikB")).toObject();
        if (!hikB.isEmpty()) {
            parts << QStringLiteral("hikB(conn=%1)")
                         .arg(hikB.value(QLatin1String("connected")).toBool() ? 1 : 0);
        }
        const QJsonObject hikC = payload.value(QLatin1String("hikC")).toObject();
        if (!hikC.isEmpty()) {
            parts << QStringLiteral("hikC(conn=%1)")
                         .arg(hikC.value(QLatin1String("connected")).toBool() ? 1 : 0);
        }
        const QJsonObject pipeline = payload.value(QLatin1String("pipeline")).toObject();
        if (!pipeline.isEmpty()) {
            parts << QStringLiteral("pipeline(st=%1)")
                         .arg(pipeline.value(QLatin1String("state")).toInt(-1));
        }
        return parts.isEmpty() ? QStringLiteral("(空)") : parts.join(QLatin1Char(' '));
    }
    if (type == QLatin1String(kStatusDevice)) {
        return QStringLiteral("onlineWord0=0x%1 faultWord0=0x%2")
            .arg(payload.value(QLatin1String("onlineWord0")).toInt(0), 4, 16, QLatin1Char('0'))
            .arg(payload.value(QLatin1String("faultWord0")).toInt(0), 4, 16, QLatin1Char('0'));
    }
    if (type == QLatin1String(kHeartbeatPing) || type == QLatin1String(kHeartbeatPong)) {
        return QStringLiteral("(空)");
    }
    if (type == QLatin1String(kEventAlarm)) {
        return QStringLiteral("level=%1 code=%2 msg=%3")
            .arg(payload.value(QLatin1String("level")).toInt(-1))
            .arg(payload.value(QLatin1String("code")).toInt(-1))
            .arg(payload.value(QLatin1String("message")).toString());
    }
    if (type == QLatin1String(kCoreHello) || type == QLatin1String(kHmiHello)) {
        return QStringLiteral("(空)");
    }
    if (payload.contains(QLatin1String("success"))) {
        return QStringLiteral("success=%1 message=%2")
            .arg(payload.value(QLatin1String("success")).toBool() ? 1 : 0)
            .arg(payload.value(QLatin1String("message")).toString());
    }
    if (!payload.isEmpty()) {
        return QStringLiteral("keys=%1").arg(payload.keys().join(QLatin1Char(',')));
    }
    return QStringLiteral("(空)");
}

}  // namespace hmi_server
}  // namespace scan_tracking
