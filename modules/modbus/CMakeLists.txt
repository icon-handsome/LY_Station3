find_package(Qt5 COMPONENTS Core SerialBus REQUIRED)

set(CMAKE_AUTOMOC ON)
add_library(scan_tracking_modbus STATIC
    include/scan_tracking/modbus/modbus_service.h
    src/modbus_service.cpp
)

target_include_directories(scan_tracking_modbus
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(scan_tracking_modbus
    PRIVATE
        Qt5::Core
        Qt5::SerialBus
    PUBLIC
        scan_tracking_common
)
