scan_tracking_require_orbbec_sdk()

add_library(scan_tracking_orbbec_gemini STATIC
    src/orbbec_gemini_service.cpp
    src/orbbec_gemini_worker.cpp
    src/orbbec_capture_io.cpp
    include/scan_tracking/orbbec_gemini/orbbec_gemini_types.h
    include/scan_tracking/orbbec_gemini/orbbec_gemini_service.h
    include/scan_tracking/orbbec_gemini/orbbec_gemini_worker.h
    include/scan_tracking/orbbec_gemini/orbbec_capture_io.h
)

set_target_properties(scan_tracking_orbbec_gemini PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_orbbec_gemini
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_orbbec_gemini
    PUBLIC
        Qt5::Core
        scan_tracking_common
    PRIVATE
        Qt5::Gui
        OrbbecSdk::OrbbecSDK
)

target_compile_features(scan_tracking_orbbec_gemini PUBLIC cxx_std_17)

add_executable(gemini335_smoke_runner
    src/gemini335_smoke_runner.cpp
)

target_link_libraries(gemini335_smoke_runner
    PRIVATE
        Qt5::Core
        scan_tracking_orbbec_gemini
)

target_compile_features(gemini335_smoke_runner PRIVATE cxx_std_17)

set_target_properties(gemini335_smoke_runner PROPERTIES
    FOLDER "tools"
)

scan_tracking_deploy_orbbec_runtime(gemini335_smoke_runner)
