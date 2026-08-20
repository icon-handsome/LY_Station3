cmake_minimum_required(VERSION 3.21)

project(ScanTrackingStation3
    VERSION 0.1.0
    DESCRIPTION "Scanning and tracking QtCore console application"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

option(SCAN_TRACKING_STRICT_TOOLCHAIN "Require the validated Windows Qt/MSVC toolchain" ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

if(SCAN_TRACKING_STRICT_TOOLCHAIN AND WIN32)
    if(NOT MSVC)
        message(FATAL_ERROR
            "ScanTracking requires MSVC on Windows. "
            "Use preset win-msvc2019-qtcore-ninja-debug or win-msvc2019-qtcore-ninja-release.")
    endif()

    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR
            "ScanTracking requires an x64 toolchain. "
            "The current compiler is not producing 64-bit binaries. "
            "Use the win-msvc2019-qtcore-ninja-* presets.")
    endif()

    string(TOLOWER "${CMAKE_CXX_COMPILER}" _scan_tracking_cxx_compiler)
    if(NOT _scan_tracking_cxx_compiler MATCHES "/msvc/14\\.29\\.30133/.*/hostx64/x64/cl\\.exe$")
        message(FATAL_ERROR
            "Unsupported MSVC compiler: ${CMAKE_CXX_COMPILER}\n"
            "Expected validated v142 compiler: "
            "C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/MSVC/14.29.30133/bin/Hostx64/x64/cl.exe\n"
            "Use preset win-msvc2019-qtcore-ninja-debug or win-msvc2019-qtcore-ninja-release.")
    endif()

    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.29 OR CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.30)
        message(FATAL_ERROR
            "Unsupported MSVC version: ${CMAKE_CXX_COMPILER_VERSION}. "
            "This project is pinned to MSVC 19.29/v142 for Qt 5.15.2 msvc2019_64.")
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(ProjectOptions)
include(OpenCvBundle)
include(MechEyeSdk)
include(OrbbecSdk)
include(LivoxSdk2)
include(HikMvsSdk)
include(CollisionDetectionSdk)

option(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION
    "Build and link LB (optical stereo marker) pose detection"
    ON
)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Network SerialPort)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Network SerialPort)

if(SCAN_TRACKING_STRICT_TOOLCHAIN AND WIN32)
    if(NOT QT_VERSION_MAJOR EQUAL 5 OR NOT Qt5Core_VERSION VERSION_EQUAL 5.15.2)
        message(FATAL_ERROR
            "Unsupported Qt version: ${Qt5Core_VERSION}. "
            "Expected Qt 5.15.2 from C:/Qt/5.15.2/msvc2019_64.")
    endif()

    string(TOLOWER "${Qt5Core_DIR}" _scan_tracking_qt5core_dir)
    if(NOT _scan_tracking_qt5core_dir MATCHES "^c:/qt/5\\.15\\.2/msvc2019_64/")
        message(FATAL_ERROR
            "Unsupported Qt path: ${Qt5Core_DIR}\n"
            "Expected C:/Qt/5.15.2/msvc2019_64. "
            "Use preset win-msvc2019-qtcore-ninja-debug or win-msvc2019-qtcore-ninja-release.")
    endif()
endif()

add_subdirectory(third_party)

add_subdirectory(common)
add_subdirectory(modules)
add_subdirectory(app)
