set(SCAN_TRACKING_VISION_SOURCES
    src/hik_mvs_sdk_runtime.cpp
    src/hik_cxp_mono_io.cpp
    src/hik_cxp_camera_service.cpp
    src/hik_mono_io.cpp
    src/lb_pose_io.cpp
    src/hik_camera_service.cpp
    src/hik_camera_c_controller.cpp
    src/hik_smart_camera_tcp_server.cpp
    src/hik_smart_camera_ftp_monitor.cpp
    src/vision_pipeline_service.cpp
    include/scan_tracking/vision/vision_types.h
    include/scan_tracking/vision/hik_mvs_sdk_runtime.h
    include/scan_tracking/vision/hik_cxp_mono_io.h
    include/scan_tracking/vision/hik_cxp_camera_service.h
    include/scan_tracking/vision/hik_mono_io.h
    include/scan_tracking/vision/lb_pose_io.h
    include/scan_tracking/vision/hik_camera_service.h
    include/scan_tracking/vision/hik_camera_c_controller.h
    include/scan_tracking/vision/hik_smart_camera_tcp_server.h
    include/scan_tracking/vision/hik_smart_camera_ftp_monitor.h
    include/scan_tracking/vision/vision_pipeline_service.h
)

if(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lb_pose_detection_adapter.cpp
        include/scan_tracking/vision/lb_pose_detection_adapter.h
    )
else()
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lb_pose_detection_adapter_stub.cpp
        include/scan_tracking/vision/lb_pose_detection_adapter.h
    )
endif()

add_library(scan_tracking_vision STATIC ${SCAN_TRACKING_VISION_SOURCES})

set_target_properties(scan_tracking_vision PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_vision
    PUBLIC
        Qt5::Core
        Qt5::Network
        scan_tracking_common
        scan_tracking_mech_eye
    PRIVATE
        HikMvsSdk::MvCameraControl
)

if(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    target_link_libraries(scan_tracking_vision
        PRIVATE
            LbPoseDetection::lb_pose_detection
    )
endif()

target_compile_features(scan_tracking_vision PUBLIC cxx_std_17)

if(BUILD_TESTING)
    find_package(Qt5 REQUIRED COMPONENTS Test)

    add_executable(scan_tracking_hik_mono_io_test
        tests/hik_mono_io_test.cpp
    )

    target_link_libraries(scan_tracking_hik_mono_io_test
        PRIVATE
            Qt5::Core
            Qt5::Test
            scan_tracking_vision
            scan_tracking_common
    )

    add_test(NAME scan_tracking_hik_mono_io_test
        COMMAND scan_tracking_hik_mono_io_test
    )

    set_target_properties(scan_tracking_hik_mono_io_test PROPERTIES
        FOLDER "tests"
    )
endif()

add_executable(scan_tracking_cxp_smoke
    tests/cxp_smoke_runner.cpp
)

target_link_libraries(scan_tracking_cxp_smoke
    PRIVATE
        Qt5::Core
        scan_tracking_vision
        scan_tracking_common
)

set_target_properties(scan_tracking_cxp_smoke PROPERTIES
    AUTOMOC ON
    FOLDER "tests"
)

scan_tracking_deploy_hik_mvs_runtime(scan_tracking_cxp_smoke)
scan_tracking_deploy_mech_eye_runtime(scan_tracking_cxp_smoke)
if(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    scan_tracking_deploy_opencv_runtime(scan_tracking_cxp_smoke)
endif()
