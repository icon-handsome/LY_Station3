include_guard(GLOBAL)

set(
    SCAN_TRACKING_MECHEYE_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Mech-Eye SDK-2.5.4/API"
    CACHE PATH
    "Path to the Mech-Eye SDK API directory"
)

function(scan_tracking_require_mech_eye_sdk)
    if(TARGET MechEyeSdk::MechEyeApi)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_MECHEYE_SDK_DIR}")
    set(_sdk_root "${_sdk_dir}/..")
    set(_include_dir "${_sdk_dir}/include")
    set(_release_lib "${_sdk_dir}/lib/MechEyeApi.lib")
    set(_debug_lib "${_sdk_dir}/lib_debug/MechEyeApid.lib")
    set(_release_dll "${_sdk_dir}/dll/MechEyeApi.dll")
    set(_debug_dll "${_sdk_dir}/dll_debug/MechEyeApid.dll")

    foreach(_required_path IN ITEMS
        "${_include_dir}/Version.h"
        "${_release_lib}"
        "${_debug_lib}"
        "${_release_dll}"
        "${_debug_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR "Mech-Eye SDK file not found: ${_required_path}")
        endif()
    endforeach()

    add_library(MechEyeSdk::MechEyeApi SHARED IMPORTED GLOBAL)
    set_target_properties(MechEyeSdk::MechEyeApi PROPERTIES
        IMPORTED_IMPLIB_RELEASE "${_release_lib}"
        IMPORTED_IMPLIB_RELWITHDEBINFO "${_release_lib}"
        IMPORTED_IMPLIB_MINSIZEREL "${_release_lib}"
        IMPORTED_IMPLIB_DEBUG "${_debug_lib}"
        IMPORTED_LOCATION_RELEASE "${_release_dll}"
        IMPORTED_LOCATION_RELWITHDEBINFO "${_release_dll}"
        IMPORTED_LOCATION_MINSIZEREL "${_release_dll}"
        IMPORTED_LOCATION_DEBUG "${_debug_dll}"
        INTERFACE_INCLUDE_DIRECTORIES "${_include_dir}"
    )

    set_property(GLOBAL PROPERTY SCAN_TRACKING_MECHEYE_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_MECHEYE_SDK_ROOT "${_sdk_root}")
endfunction()

function(scan_tracking_deploy_mech_eye_runtime target_name)
    scan_tracking_require_mech_eye_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_MECHEYE_SDK_DIR)
    get_property(_sdk_root GLOBAL PROPERTY SCAN_TRACKING_MECHEYE_SDK_ROOT)

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_sdk_dir}/dll_debug/MechEyeApid.dll,${_sdk_dir}/dll/MechEyeApi.dll>"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_sdk_dir}/dll_debug/MechEyeApiWrapperd.dll,${_sdk_dir}/dll/MechEyeApiWrapper.dll>"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "$<IF:$<CONFIG:Debug>,${_sdk_dir}/dll_debug,${_sdk_dir}/dll>"
            "$<TARGET_FILE_DIR:${target_name}>/mech_eye_api"
        COMMENT "Deploying Mech-Eye SDK runtime"
    )

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_root};${_sdk_dir}/dll;${_sdk_dir}/dll_debug;%PATH%"
        )
    endif()
endfunction()
