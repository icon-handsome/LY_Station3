#include "scan_tracking/mech_eye/mech_eye_nano_smoke_test.h"

#include <exception>
#include <vector>

#include <QtCore/QTextStream>
#include <QtCore/QString>
#include <QtCore/QDir>
#include <QtCore/QDateTime>
#include <QtGui/QImage>

#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/Frame2D.h"

namespace scan_tracking {
namespace mech_eye {


/* 执行 Mech-Eye Nano 烟雾测试，按发现、连接、采集、保存、断开五步完成验证。 */
void MechEyeNanoSmokeTest::run() {
    QTextStream stream(stdout);
    stream << "\n--- [ Mech-Eye Nano Smoke Test Started ] ---\n";
    try {
        stream << "1. Discovering cameras...\n";
        std::vector<mmind::eye::CameraInfo> camera_list = mmind::eye::Camera::discoverCameras();
        if (camera_list.empty()) {
            stream << "   [ERROR] No cameras found.\n--- [ Smoke Test Finished ] ---\n\n";
            stream.flush();
            return;
        }

        stream << "   Found " << static_cast<qulonglong>(camera_list.size()) << " camera(s).\n";
        for (size_t i = 0; i < camera_list.size(); ++i) {
            stream << "   [" << i << "] Model: " << QString::fromStdString(camera_list[i].model)
                   << ", SN: " << QString::fromStdString(camera_list[i].serialNumber)
                   << ", IP: " << QString::fromStdString(camera_list[i].ipAddress) << "\n";
        }

        stream << "\n2. Connecting to the first camera (" << QString::fromStdString(camera_list[0].serialNumber) << ")...\n";
        mmind::eye::Camera camera;
        auto status = camera.connect(camera_list[0]);
        if (!status.isOK()) {
            stream << "   [ERROR] Failed to connect: " << QString::fromStdString(status.errorDescription) << "\n";
            stream << "--- [ Smoke Test Finished ] ---\n\n";
            stream.flush();
            return;
        }
        stream << "   [SUCCESS] Connected.\n";

        stream << "\n3. Triggering 2D capture...\n";
        mmind::eye::Frame2D frame_2d;
        status = camera.capture2D(frame_2d);
        if (!status.isOK()) {
            stream << "   [ERROR] Capture 2D failed: " << QString::fromStdString(status.errorDescription) << "\n";
        } else {
            stream << "   [SUCCESS] Captured 2D image!\n";
            
           QString out_dir_path = QDir::currentPath() + "/output/captures";
            QDir out_dir;
            if (!out_dir.mkpath(out_dir_path)) {
                stream << "   [ERROR] Failed to create directory: " << out_dir_path << "\n";
            } else {
                QString file_name = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz") + "_2DImage.png";
                QString save_path = out_dir_path + "/" + file_name;
                
                QImage img;
                if (frame_2d.colorType() == mmind::eye::ColorTypeOf2DCamera::Monochrome) {
                    mmind::eye::GrayScale2DImage grayImage = frame_2d.getGrayScaleImage();
                    uint8_t* data = (uint8_t*)grayImage.data();
                    int width = grayImage.width();
                    int height = grayImage.height();
                    if (data && width > 0 && height > 0) {
                        img = QImage(data, width, height, width, QImage::Format_Grayscale8);
                    }
                } else if (frame_2d.colorType() == mmind::eye::ColorTypeOf2DCamera::Color) {
                    mmind::eye::Color2DImage colorImage = frame_2d.getColorImage();
                    uint8_t* data = (uint8_t*)colorImage.data();
                    int width = colorImage.width();
                    int height = colorImage.height();
                    if (data && width > 0 && height > 0) {
                        img = QImage(data, width, height, width * 3, QImage::Format_RGB888);
                    }
                }

                if (!img.isNull()) {
                    if (img.save(save_path)) {
                        stream << "   [SUCCESS] Saved 2D image to: " << QDir::toNativeSeparators(save_path) << "\n";
                    } else {
                        stream << "   [ERROR]s Failed to save 2D image to: " << QDir::toNativeSeparators(save_path) << "\n";
                    }
                } else {
                    stream << "   [ERROR] Failed to convert 2D image data to QImage.\n";
                }
            }
        }

        stream << "\n4. Disconnecting...\n";
        camera.disconnect();
        stream << "   [SUCCESS] Disconnected.\n";

    } catch (const std::exception& e) {
        stream << "   [EXCEPTION] Standard exception: " << QString::fromLocal8Bit(e.what()) << "\n";
    } catch (...) {
        stream << "   [EXCEPTION] Unknown exception occurred.\n";
    }
    stream << "--- [ Mech-Eye Nano Smoke Test Finished ] ---\n\n";
    stream.flush();
}

}  // namespace mech_eye
}  // namespace scan_tracking
