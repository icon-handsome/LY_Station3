scan_tracking_require_mech_eye_sdk()

add_library(scan_tracking_mech_eye STATIC
    src/mech_eye_service.cpp
    src/mech_eye_worker.cpp
    src/mech_eye_sdk_seh.cpp
    src/point_cloud_io.cpp
    src/point_cloud_processor.cpp
    src/mech_eye_nano_smoke_test.cpp
    include/scan_tracking/mech_eye/mech_eye_types.h
    include/scan_tracking/mech_eye/mech_eye_service.h
    include/scan_tracking/mech_eye/mech_eye_worker.h
    include/scan_tracking/mech_eye/mech_eye_sdk_seh.h
    include/scan_tracking/mech_eye/point_cloud_io.h
    include/scan_tracking/mech_eye/point_cloud_processor.h
)

set_target_properties(scan_tracking_mech_eye PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_mech_eye
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/eigen-3.4.0
)

target_link_libraries(scan_tracking_mech_eye
    PUBLIC
        Qt5::Core
        Qt5::Gui
        Qt5::Network
        scan_tracking_common
    PRIVATE
        MechEyeSdk::MechEyeApi
)

target_compile_features(scan_tracking_mech_eye PUBLIC cxx_std_17)

if(WIN32)
    target_link_libraries(scan_tracking_mech_eye PRIVATE iphlpapi ws2_32)
endif()

if(BUILD_TESTING)
    find_package(Qt5 REQUIRED COMPONENTS Test)

    add_executable(scan_tracking_point_cloud_io_test
        tests/point_cloud_io_test.cpp
    )

    target_link_libraries(scan_tracking_point_cloud_io_test
        PRIVATE
            Qt5::Core
            Qt5::Test
            scan_tracking_mech_eye
            scan_tracking_common
    )

    add_test(NAME scan_tracking_point_cloud_io_test
        COMMAND scan_tracking_point_cloud_io_test
    )

    set_target_properties(scan_tracking_point_cloud_io_test PROPERTIES
        FOLDER "tests"
    )
endif()
