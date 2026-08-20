include_guard(GLOBAL)

set(
    SCAN_TRACKING_HIK_MVS_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MVS"
    CACHE PATH
    "Path to the local HIK MVS SDK bundle"
)

function(scan_tracking_require_hik_mvs_sdk)
    if(TARGET HikMvsSdk::MvCameraControl)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_HIK_MVS_SDK_DIR}")

    set(_include_candidates
        "${_sdk_dir}/Development/Includes"
        "${_sdk_dir}/include"
    )
    set(_lib_candidates
        "${_sdk_dir}/Development/Libraries/win64"
        "${_sdk_dir}/lib/win64"
    )
    foreach(_candidate IN LISTS _include_candidates)
        if(EXISTS "${_candidate}/MvCameraControl.h" AND EXISTS "${_candidate}/CameraParams.h")
            set(_include_dir "${_candidate}")
            break()
        endif()
    endforeach()

    foreach(_candidate IN LISTS _lib_candidates)
        if(EXISTS "${_candidate}/MvCameraControl.lib")
            set(_lib_dir "${_candidate}")
            break()
        endif()
    endforeach()

    set(_runtime_dir "")
    if(EXISTS "${_sdk_dir}/Development/Bin/win64/MvCameraControl.dll")
        set(_runtime_dir "${_sdk_dir}/Development/Bin/win64")
    elseif(EXISTS "${_sdk_dir}/MVS/Runtime/Win64_x64/MvCameraControl.dll")
        set(_runtime_dir "${_sdk_dir}/MVS/Runtime/Win64_x64")
    elseif(EXISTS "${_sdk_dir}/runtime/Win64_x64/MvCameraControl.dll")
        set(_runtime_dir "${_sdk_dir}/runtime/Win64_x64")
    endif()

    if(NOT DEFINED _include_dir OR NOT DEFINED _lib_dir)
        message(FATAL_ERROR
            "HIK MVS SDK layout not recognized in: ${_sdk_dir}. "
            "Expected Development/Includes, Development/Libraries/win64 and Development/Bin/win64, "
            "or MVS/Runtime/Win64_x64, "
            "or the older include/lib/win64/runtime/Win64_x64 layout."
        )
    endif()

    set(_import_lib "${_lib_dir}/MvCameraControl.lib")
    set(_runtime_dll "")
    if(NOT _runtime_dir STREQUAL "")
        set(_runtime_dll "${_runtime_dir}/MvCameraControl.dll")
    endif()

    foreach(_required_path IN ITEMS
        "${_include_dir}/MvCameraControl.h"
        "${_include_dir}/CameraParams.h"
        "${_import_lib}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR "HIK MVS SDK file not found: ${_required_path}")
        endif()
    endforeach()
    if(NOT _runtime_dll STREQUAL "" AND NOT EXISTS "${_runtime_dll}")
        message(FATAL_ERROR "HIK MVS SDK file not found: ${_runtime_dll}")
    endif()

    add_library(HikMvsSdk::MvCameraControl UNKNOWN IMPORTED GLOBAL)
    set_target_properties(HikMvsSdk::MvCameraControl PROPERTIES
        IMPORTED_IMPLIB "${_import_lib}"
        INTERFACE_INCLUDE_DIRECTORIES "${_include_dir}"
        IMPORTED_LOCATION "${_import_lib}"
    )

    set_property(GLOBAL PROPERTY SCAN_TRACKING_HIK_MVS_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_HIK_MVS_SDK_RUNTIME_DIR "${_runtime_dir}")
endfunction()

function(scan_tracking_deploy_hik_mvs_runtime target_name)
    scan_tracking_require_hik_mvs_sdk()

    get_property(_runtime_dir GLOBAL PROPERTY SCAN_TRACKING_HIK_MVS_SDK_RUNTIME_DIR)

    if(NOT _runtime_dir STREQUAL "")
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_runtime_dir}"
                "$<TARGET_FILE_DIR:${target_name}>"
            COMMENT "Deploying HIK MVS runtime"
        )

        if(MSVC)
            set(_mvs_gentl_default "C:/Program Files (x86)/Common Files/MVS/Runtime/Win64_x64")
            set_property(TARGET ${target_name} APPEND PROPERTY
                VS_DEBUGGER_ENVIRONMENT
                "PATH=${_runtime_dir};%PATH%\nGENICAM_GENTL64_PATH=${_mvs_gentl_default};${_runtime_dir}"
            )
        endif()
    endif()
endfunction()
