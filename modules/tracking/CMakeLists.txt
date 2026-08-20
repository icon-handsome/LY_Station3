add_library(scan_tracking_tracking STATIC
    src/lb_pose_check.cpp
    include/scan_tracking/tracking/lb_pose_check.h
)

target_include_directories(scan_tracking_tracking
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(scan_tracking_tracking PUBLIC cxx_std_17)

target_link_libraries(scan_tracking_tracking
    PUBLIC
        Qt5::Core
        scan_tracking_common
    PRIVATE
        LbPoseDetection::lb_pose_detection
)

set_target_properties(scan_tracking_tracking PROPERTIES
    FOLDER "modules"
)
