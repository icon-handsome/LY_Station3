scan_tracking_require_livox_sdk2()

add_library(scan_tracking_livox_mid360 STATIC
    src/livox_mid360_service.cpp
    src/livox_mid360_worker.cpp
    include/scan_tracking/livox_mid360/livox_mid360_types.h
    include/scan_tracking/livox_mid360/livox_mid360_service.h
    include/scan_tracking/livox_mid360/livox_mid360_worker.h
)

set_target_properties(scan_tracking_livox_mid360 PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_livox_mid360
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_livox_mid360
    PUBLIC
        Qt5::Core
        scan_tracking_common
    PRIVATE
        LivoxSdk2::livox_lidar_sdk_static
)

target_compile_features(scan_tracking_livox_mid360 PUBLIC cxx_std_17)
