#include "scan_tracking/mech_eye/point_cloud_processor.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <exception>
#include <new>

namespace scan_tracking::mech_eye {
namespace {

Eigen::Matrix4f toEigenRowMajor(const std::array<float, 16>& m)
{
    Eigen::Matrix4f out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out(r, c) = m[static_cast<std::size_t>(r * 4 + c)];
        }
    }
    return out;
}

}  // namespace

std::mutex& pointCloudAlgorithmMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::array<float, 16> multiplyRowMajor4x4(
    const std::array<float, 16>& left,
    const std::array<float, 16>& right)
{
    const Eigen::Matrix4f a = toEigenRowMajor(left);
    const Eigen::Matrix4f b = toEigenRowMajor(right);
    const Eigen::Matrix4f c = a * b;
    std::array<float, 16> out{};
    for (int r = 0; r < 4; ++r) {
        for (int cIdx = 0; cIdx < 4; ++cIdx) {
            out[static_cast<std::size_t>(r * 4 + cIdx)] = c(r, cIdx);
        }
    }
    return out;
}

bool transformPointCloudFrame(
    const PointCloudFrame& input,
    const std::array<float, 16>& calibrationMatrixT0Prime,
    const std::array<float, 16>& stereoTrackingMatrixT,
    PointCloudFrame* output,
    QString* message)
{
    if (output == nullptr) {
        if (message != nullptr) {
            *message = QStringLiteral("output 为空。");
        }
        return false;
    }
    if (!input.isValid() || input.pointsXYZ == nullptr) {
        if (message != nullptr) {
            *message = QStringLiteral("输入点云无效。");
        }
        return false;
    }

    try {
        std::lock_guard<std::mutex> lock(pointCloudAlgorithmMutex());

        const Eigen::Matrix4f combined =
            toEigenRowMajor(calibrationMatrixT0Prime) * toEigenRowMajor(stereoTrackingMatrixT);

        auto outPoints = std::make_shared<std::vector<float>>();
        outPoints->resize(input.pointsXYZ->size());

        const auto& in = *input.pointsXYZ;
        auto& out = *outPoints;
        const int count = input.pointCount > 0
            ? input.pointCount
            : static_cast<int>(in.size() / 3);

        for (int i = 0; i < count; ++i) {
            const std::size_t base = static_cast<std::size_t>(i) * 3;
            if (base + 2 >= in.size()) {
                break;
            }
            Eigen::Vector4f p(in[base], in[base + 1], in[base + 2], 1.0f);
            // 与第一工位 PCL transformPointCloud 一致：列向量 p' = M * p
            const Eigen::Vector4f q = combined * p;
            out[base] = q.x();
            out[base + 1] = q.y();
            out[base + 2] = q.z();
        }

        *output = input;
        output->pointsXYZ = outPoints;
        output->pointCount = count;
        if (message != nullptr) {
            *message = QStringLiteral("点云变换完成，点数=%1").arg(count);
        }
        return true;
    } catch (const std::bad_alloc&) {
        // LB worker 等裸 std::thread 上未捕获异常会 terminate；OOM 降级为失败。
        if (message != nullptr) {
            *message = QStringLiteral("点云变换内存不足。");
        }
        return false;
    } catch (const std::exception& ex) {
        if (message != nullptr) {
            *message = QStringLiteral("点云变换异常：%1").arg(QString::fromUtf8(ex.what()));
        }
        return false;
    } catch (...) {
        if (message != nullptr) {
            *message = QStringLiteral("点云变换发生未知异常。");
        }
        return false;
    }
}

}  // namespace scan_tracking::mech_eye
