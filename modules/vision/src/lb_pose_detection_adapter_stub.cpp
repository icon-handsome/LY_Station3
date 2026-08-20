#include "scan_tracking/vision/lb_pose_detection_adapter.h"

// 未链接 TR_Mark 库时的占位实现，invoked=false 表示算法未真正执行
namespace scan_tracking::vision {

LbPoseResult runLbPoseDetection(
    const HikMonoFrame& leftFrame,
    const HikMonoFrame& rightFrame,
    const scan_tracking::common::LbPoseConfig&)
{
    LbPoseResult result;
    result.invoked = false;
    result.leftImageWidth = leftFrame.width;
    result.leftImageHeight = leftFrame.height;
    result.rightImageWidth = rightFrame.width;
    result.rightImageHeight = rightFrame.height;
    result.success = false;
    result.message = QStringLiteral("此构建版本中 LB 位姿检测已禁用。");
    return result;
}

}  // namespace scan_tracking::vision
