#pragma once

namespace scan_tracking {
namespace mech_eye {

/* Mech-Eye Nano 烟雾测试入口
 * 该类用于快速验证相机发现、连接与 2D 采集流程是否可用。
 */
class MechEyeNanoSmokeTest {
public:
    /* 执行一次完整的烟雾测试流程 */
    static void run();
};

}  // namespace mech_eye
}  // namespace scan_tracking