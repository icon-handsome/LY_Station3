#include "scan_tracking/vision/hik_mono_io.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QtTest/QtTest>

using namespace scan_tracking::vision;

class HikMonoIoTest : public QObject {
    Q_OBJECT

private slots:
    void roundTripSaveBmp();
};

void HikMonoIoTest::roundTripSaveBmp()
{
    HikMonoFrame frame;
    frame.width = 2;
    frame.height = 2;
    frame.stride = 2;
    frame.pixels = std::make_shared<std::vector<std::uint8_t>>(std::initializer_list<std::uint8_t>{
        10, 20,
        30, 40,
    });

    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString path =
        buildSegmentHikMonoPath(tempDir.path(), 1, QStringLiteral("arm"), 2, QStringLiteral("hikA"));
    QVERIFY(path.contains(QStringLiteral("path_1")));
    QVERIFY(path.contains(QStringLiteral("arm")));
    QVERIFY(path.endsWith(QStringLiteral("Path1_Arm_hikA_2.bmp")));
    QVERIFY(saveHikMonoFrameToBmp(frame, path));
    QVERIFY(QFile::exists(path));

    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly));
    const QByteArray content = file.readAll();
    QVERIFY(content.size() >= 2);
    QCOMPARE(content.at(0), char('B'));
    QCOMPARE(content.at(1), char('M'));
}

QTEST_MAIN(HikMonoIoTest)
#include "hik_mono_io_test.moc"
