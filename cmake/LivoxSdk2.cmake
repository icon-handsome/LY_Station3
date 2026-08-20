include_guard(GLOBAL)

set(
    SCAN_TRACKING_LIVOX_SDK_ROOT
    "${CMAKE_SOURCE_DIR}/third_party/Livox-SDK2"
    CACHE PATH
    "Path to the Livox SDK2 installation root (contains include/, lib/, bin/)"
)

function(scan_tracking_require_livox_sdk2)
    if(TARGET LivoxSdk2::livox_lidar_sdk_static)
        return()
    endif()

    set(_sdk_root "${SCAN_TRACKING_LIVOX_SDK_ROOT}")
    set(_include_dir "${_sdk_root}/include")
    set(_header "${_include_dir}/livox_lidar_api.h")
    set(_release_lib "${_sdk_root}/lib/livox_lidar_sdk_static.lib")
    set(_debug_lib "${_sdk_root}/lib/livox_lidar_sdk_staticd.lib")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_debug_lib}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR
                "Livox SDK2 file not found: ${_required_path}\n"
                "Build third_party/Livox-SDK2-source with MSVC 14.29 for both Release and Debug, "
                "then install libs to third_party/Livox-SDK2/lib/ "
                "(livox_lidar_sdk_static.lib / livox_lidar_sdk_staticd.lib).")
        endif()
    endforeach()

    add_library(LivoxSdk2::livox_lidar_sdk_static STATIC IMPORTED GLOBAL)
    set_target_properties(LivoxSdk2::livox_lidar_sdk_static PROPERTIES
        IMPORTED_LOCATION_RELEASE "${_release_lib}"
        IMPORTED_LOCATION_RELWITHDEBINFO "${_release_lib}"
        IMPORTED_LOCATION_MINSIZEREL "${_release_lib}"
        IMPORTED_LOCATION_DEBUG "${_debug_lib}"
        INTERFACE_INCLUDE_DIRECTORIES "${_include_dir}"
    )

    if(WIN32)
        set_property(TARGET LivoxSdk2::livox_lidar_sdk_static APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES ws2_32)
    endif()

    set_property(GLOBAL PROPERTY SCAN_TRACKING_LIVOX_SDK_ROOT "${_sdk_root}")
endfunction()

function(scan_tracking_deploy_livox_runtime target_name)
    scan_tracking_require_livox_sdk2()

    get_property(_sdk_root GLOBAL PROPERTY SCAN_TRACKING_LIVOX_SDK_ROOT)
    set(_config_file "${_sdk_root}/bin/mid360_config.json")

    if(EXISTS "${_config_file}")
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/livox"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_file}"
                "$<TARGET_FILE_DIR:${target_name}>/livox/mid360_config.json"
            COMMENT "Deploying Livox MID-360 config"
        )
    endif()
endfunction()
