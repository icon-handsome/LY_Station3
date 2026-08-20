include_guard(GLOBAL)

set(
    SCAN_TRACKING_ORBBEC_SDK_ROOT
    "C:/Program Files/OrbbecSDK 2.8.6"
    CACHE PATH
    "Path to the Orbbec SDK installation root (contains include/, lib/, bin/)"
)

function(scan_tracking_require_orbbec_sdk)
    if(TARGET OrbbecSdk::OrbbecSDK)
        return()
    endif()

    set(_sdk_root "${SCAN_TRACKING_ORBBEC_SDK_ROOT}")
    set(_include_dir "${_sdk_root}/include")
    set(_header "${_include_dir}/libobsensor/ObSensor.hpp")
    set(_release_lib "${_sdk_root}/lib/OrbbecSDK.lib")
    set(_release_dll "${_sdk_root}/bin/OrbbecSDK.dll")
    set(_extensions_dir "${_sdk_root}/bin/extensions")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_release_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR "Orbbec SDK file not found: ${_required_path}")
        endif()
    endforeach()

    if(NOT EXISTS "${_extensions_dir}")
        message(WARNING "Orbbec SDK extensions directory not found: ${_extensions_dir}")
    endif()

    add_library(OrbbecSdk::OrbbecSDK SHARED IMPORTED GLOBAL)
    set_target_properties(OrbbecSdk::OrbbecSDK PROPERTIES
        IMPORTED_IMPLIB_RELEASE "${_release_lib}"
        IMPORTED_IMPLIB_RELWITHDEBINFO "${_release_lib}"
        IMPORTED_IMPLIB_MINSIZEREL "${_release_lib}"
        IMPORTED_IMPLIB_DEBUG "${_release_lib}"
        IMPORTED_LOCATION_RELEASE "${_release_dll}"
        IMPORTED_LOCATION_RELWITHDEBINFO "${_release_dll}"
        IMPORTED_LOCATION_MINSIZEREL "${_release_dll}"
        IMPORTED_LOCATION_DEBUG "${_release_dll}"
        INTERFACE_INCLUDE_DIRECTORIES "${_include_dir}"
    )

    set_property(GLOBAL PROPERTY SCAN_TRACKING_ORBBEC_SDK_ROOT "${_sdk_root}")
endfunction()

function(scan_tracking_deploy_orbbec_runtime target_name)
    scan_tracking_require_orbbec_sdk()

    get_property(_sdk_root GLOBAL PROPERTY SCAN_TRACKING_ORBBEC_SDK_ROOT)
    set(_release_dll "${_sdk_root}/bin/OrbbecSDK.dll")
    set(_extensions_dir "${_sdk_root}/bin/extensions")

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_release_dll}"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMENT "Deploying Orbbec SDK runtime"
    )

    if(EXISTS "${_extensions_dir}")
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_extensions_dir}"
                "$<TARGET_FILE_DIR:${target_name}>/extensions"
            COMMENT "Deploying Orbbec SDK extensions"
        )
    endif()

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_root}/bin;%PATH%"
        )
    endif()
endfunction()
