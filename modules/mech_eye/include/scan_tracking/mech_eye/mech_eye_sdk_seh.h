#pragma once

// Mech-Eye SDK 在多网卡工控机上可能触发原生崩溃（Access Violation）。
// C++ try/catch 拦不住；Windows 下用 SEH 兜住调用点。
// 注意：捕获 AV 后 SDK 全局态可能已损坏——应标记进程级隔离并避免继续 new/connect。

#include <string>
#include <vector>

#include "ErrorStatus.h"
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/CameraProperties.h"
#include "area_scan_3d_camera/Frame2D.h"
#include "area_scan_3d_camera/Frame2DAnd3D.h"
#include "area_scan_3d_camera/Frame3D.h"

namespace scan_tracking {
namespace mech_eye {
namespace sdk_seh {

/// Windows STATUS_ACCESS_VIOLATION
constexpr unsigned kSehAccessViolation = 0xC0000005u;
/// MSVC C++ 异常（RaiseException），常见于 AV 后 SDK 脏状态下的后续构造
constexpr unsigned kSehCppException = 0xE06D7363u;

bool isAccessViolationSeh(unsigned sehCode);
bool isSdkProcessIsolated();
void markSdkProcessIsolated();

/// 在 SEH 下执行无返回值回调；@return 0 成功，非 0 为异常码
unsigned invokeVoid(void (*fn)(void*), void* ctx);

/// @return 0=未发生 SEH；非 0=Windows 异常码（出参相机指针保证为 nullptr，勿 delete 半残对象）
unsigned createCamera(mmind::eye::Camera** outCamera);

unsigned connectByIp(
    mmind::eye::Camera* camera,
    const std::string& ip,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned connectByInfo(
    mmind::eye::Camera* camera,
    const mmind::eye::CameraInfo& info,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned discoverCameras(
    unsigned timeoutMs,
    std::vector<mmind::eye::CameraInfo>* outList);

unsigned disconnect(mmind::eye::Camera* camera);

unsigned getCameraInfo(
    mmind::eye::Camera* camera,
    mmind::eye::CameraInfo* outInfo,
    mmind::eye::ErrorStatus* outStatus);

unsigned setHeartbeatInterval(mmind::eye::Camera* camera, int intervalMs);

unsigned capture2D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned capture3D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned capture2DAnd3D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2DAnd3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

}  // namespace sdk_seh
}  // namespace mech_eye
}  // namespace scan_tracking
