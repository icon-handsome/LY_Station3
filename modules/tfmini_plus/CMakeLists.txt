add_library(scan_tracking_tfmini_plus STATIC
    src/tfmini_plus_service.cpp
    src/tfmini_plus_worker.cpp
    include/scan_tracking/tfmini_plus/tfmini_plus_service.h
    include/scan_tracking/tfmini_plus/tfmini_plus_types.h
    include/scan_tracking/tfmini_plus/tfmini_plus_worker.h
)

set_target_properties(scan_tracking_tfmini_plus PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_tfmini_plus
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_tfmini_plus
    PUBLIC
        Qt5::Core
        scan_tracking_common
    PRIVATE
        Qt5::SerialPort
)

target_compile_features(scan_tracking_tfmini_plus PUBLIC cxx_std_17)
