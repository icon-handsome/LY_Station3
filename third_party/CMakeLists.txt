set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    add_subdirectory(LB)
endif()

# Large header-only SDK trees: OFF by default to keep VS Solution Explorer lean.
option(SCAN_TRACKING_SHOW_EIGEN_IN_IDE
    "Expose third_party/eigen-3.4.0 in the IDE (INTERFACE + resource targets)"
    OFF
)
option(SCAN_TRACKING_SHOW_MECHEYE_SDK_IN_IDE
    "Expose third_party/Mech-Eye SDK-2.5.4 in the IDE (INTERFACE + resource targets)"
    OFF
)

function(scan_tracking_add_ide_resource_target target_name root_dir prefix)
    set(resource_files ${ARGN})
    if(resource_files)
        add_custom_target(${target_name} SOURCES ${resource_files})
        source_group(TREE "${root_dir}" PREFIX "${prefix}" FILES ${resource_files})
        set_target_properties(${target_name} PROPERTIES FOLDER "third_party")
    endif()
endfunction()

if(SCAN_TRACKING_SHOW_EIGEN_IN_IDE)
    file(GLOB_RECURSE SCAN_TRACKING_EIGEN_HEADERS CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/Eigen/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/Eigen/*.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/Eigen/*.hxx"
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/Eigen/*.inl"
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/unsupported/Eigen/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/unsupported/Eigen/*.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/unsupported/Eigen/*.hxx"
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0/unsupported/Eigen/*.inl"
    )

    add_library(scan_tracking_third_party_eigen INTERFACE)
    target_sources(scan_tracking_third_party_eigen INTERFACE ${SCAN_TRACKING_EIGEN_HEADERS})
    target_include_directories(scan_tracking_third_party_eigen INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0" PREFIX "third_party/eigen-3.4.0" FILES ${SCAN_TRACKING_EIGEN_HEADERS})
    set_target_properties(scan_tracking_third_party_eigen PROPERTIES FOLDER "third_party")
    scan_tracking_add_ide_resource_target(
        scan_tracking_third_party_eigen_resources
        "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0"
        "third_party/eigen-3.4.0"
        ${SCAN_TRACKING_EIGEN_HEADERS}
    )
endif()

if(SCAN_TRACKING_SHOW_MECHEYE_SDK_IN_IDE)
    file(GLOB_RECURSE SCAN_TRACKING_MECHEYE_HEADERS CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/include/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/include/*.hpp"
    )

    add_library(scan_tracking_third_party_mecheye INTERFACE)
    target_sources(scan_tracking_third_party_mecheye INTERFACE ${SCAN_TRACKING_MECHEYE_HEADERS})
    target_include_directories(scan_tracking_third_party_mecheye INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/include")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/include" PREFIX "third_party/Mech-Eye SDK-2.5.4/API/include" FILES ${SCAN_TRACKING_MECHEYE_HEADERS})
    set_target_properties(scan_tracking_third_party_mecheye PROPERTIES FOLDER "third_party")
    scan_tracking_add_ide_resource_target(
        scan_tracking_third_party_mecheye_resources
        "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4"
        "third_party/Mech-Eye SDK-2.5.4"
        ${SCAN_TRACKING_MECHEYE_HEADERS}
    )

    file(GLOB_RECURSE SCAN_TRACKING_MECHEYE_SAMPLES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/samples/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/samples/*.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/samples/*.cpp"
    )
    if(SCAN_TRACKING_MECHEYE_SAMPLES)
        add_library(scan_tracking_third_party_mecheye_samples INTERFACE)
        target_sources(scan_tracking_third_party_mecheye_samples INTERFACE ${SCAN_TRACKING_MECHEYE_SAMPLES})
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/samples" PREFIX "third_party/Mech-Eye SDK-2.5.4/API/samples" FILES ${SCAN_TRACKING_MECHEYE_SAMPLES})
        set_target_properties(scan_tracking_third_party_mecheye_samples PROPERTIES FOLDER "third_party")
        scan_tracking_add_ide_resource_target(
            scan_tracking_third_party_mecheye_sample_resources
            "${CMAKE_CURRENT_SOURCE_DIR}/Mech-Eye SDK-2.5.4/API/samples"
            "third_party/Mech-Eye SDK-2.5.4/API/samples"
            ${SCAN_TRACKING_MECHEYE_SAMPLES}
        )
    endif()
endif()

file(GLOB_RECURSE SCAN_TRACKING_HIK_MVS_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Includes/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Includes/*.hpp"
)

add_library(scan_tracking_third_party_hik_mvs INTERFACE)
target_sources(scan_tracking_third_party_hik_mvs INTERFACE ${SCAN_TRACKING_HIK_MVS_HEADERS})
target_include_directories(scan_tracking_third_party_hik_mvs INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Includes")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Includes" PREFIX "third_party/MVS/Development/Includes" FILES ${SCAN_TRACKING_HIK_MVS_HEADERS})
set_target_properties(scan_tracking_third_party_hik_mvs PROPERTIES FOLDER "third_party")
scan_tracking_add_ide_resource_target(
    scan_tracking_third_party_hik_mvs_resources
    "${CMAKE_CURRENT_SOURCE_DIR}/MVS"
    "third_party/MVS"
    ${SCAN_TRACKING_HIK_MVS_HEADERS}
)

file(GLOB_RECURSE SCAN_TRACKING_HIK_MVS_SAMPLES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Samples/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Samples/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Samples/*.cpp"
)
if(SCAN_TRACKING_HIK_MVS_SAMPLES)
    add_library(scan_tracking_third_party_hik_mvs_samples INTERFACE)
    target_sources(scan_tracking_third_party_hik_mvs_samples INTERFACE ${SCAN_TRACKING_HIK_MVS_SAMPLES})
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Samples" PREFIX "third_party/MVS/Development/Samples" FILES ${SCAN_TRACKING_HIK_MVS_SAMPLES})
    set_target_properties(scan_tracking_third_party_hik_mvs_samples PROPERTIES FOLDER "third_party")
    scan_tracking_add_ide_resource_target(
        scan_tracking_third_party_hik_mvs_sample_resources
        "${CMAKE_CURRENT_SOURCE_DIR}/MVS/Development/Samples"
        "third_party/MVS/Development/Samples"
        ${SCAN_TRACKING_HIK_MVS_SAMPLES}
    )
endif()

file(GLOB_RECURSE SCAN_TRACKING_THIRD_PARTY_MISC_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/MVS/*"
)
# 排除 SDK 自带的 C#/VB/C++ 示例工程，否则 VS 会把 .csproj 纳入生成并触发 MSB3644（.NET Framework 4.0）
list(FILTER SCAN_TRACKING_THIRD_PARTY_MISC_FILES EXCLUDE REGEX
    "\\.(csproj|vbproj|vcxproj|vcproj|sln|cs|vb|fsproj|resx)$")
if(SCAN_TRACKING_THIRD_PARTY_MISC_FILES)
    scan_tracking_add_ide_resource_target(
        scan_tracking_third_party_misc_resources
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "third_party"
        ${SCAN_TRACKING_THIRD_PARTY_MISC_FILES}
    )
endif()

