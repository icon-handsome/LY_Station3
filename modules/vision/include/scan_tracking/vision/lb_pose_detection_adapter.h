#pragma once

// LB（封头段）位姿检测适配层。
//
// 基于海康 CXP 双目灰度图，调用 third_party/LB（TR_Mark + AppConfig）完成：
//   1. 立体标定 + 极线约束下的 3D 点云重建（TR_INSPECT_3D_Recon_Marker）
//   2. 模板点云几何哈希匹配，输出 4×4 Rt_global（FastGeoHash / Get_Track_Pose）
//      IPC 拼接约定：Rt_global 作为 T0（p' = p × T0），不再作为段位姿 T 与 JSON T0' 相乘。
//
// 由 VisionPipelineService 在封头段（needMechEye2D=false）的后台线程中调用。
// 未链接 TR_Mark 的构建使用 lb_pose_detection_adapter_stub.cpp 中的空实现。

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

/* 执行 LB 位姿检测（同步、阻塞调用，应在后台线程中使用）
 *
 * @param leftFrame  CXP 左目 Mono8 灰度帧（现场 = 相机 B，对应 track_config I1）
 * @param rightFrame CXP 右目 Mono8 灰度帧（现场 = 相机 A，对应 track_config I2）
 * @param config     [LbPose] trackConfigFile 与可选 templateFile 覆盖
 * @return LbPoseResult，含 poseMatrix（4×4 Rt_global，IPC 侧作 T0）、framePointCount 及错误说明
 */
LbPoseResult runLbPoseDetection(
    const HikMonoFrame& leftFrame,
    const HikMonoFrame& rightFrame,
    const scan_tracking::common::LbPoseConfig& config);

}  // namespace vision
}  // namespace scan_tracking
