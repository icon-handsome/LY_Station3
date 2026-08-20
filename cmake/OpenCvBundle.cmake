include_guard(GLOBAL)

set(_scan_tracking_opencv_force_cache "")

if(DEFINED SCAN_TRACKING_OPENCV_DIR AND NOT "${SCAN_TRACKING_OPENCV_DIR}" STREQUAL "")
    get_filename_component(_scan_tracking_opencv_candidate "${SCAN_TRACKING_OPENCV_DIR}" ABSOLUTE)
    if(EXISTS "${_scan_tracking_opencv_candidate}/OpenCVConfig.cmake")
        # Preset passes the import-lib directory (.../x64/vc15/lib).
        set(_scan_tracking_opencv_lib_dir "${_scan_tracking_opencv_candidate}")
        get_filename_component(_scan_tracking_opencv_build_root "${_scan_tracking_opencv_lib_dir}/../../.." ABSOLUTE)
        set(_scan_tracking_opencv_force_cache FORCE)
    elseif(EXISTS "${_scan_tracking_opencv_candidate}/x64/vc15/lib/OpenCVConfig.cmake")
        # Caller passed the OpenCV build root (.../opencv/build).
        set(_scan_tracking_opencv_build_root "${_scan_tracking_opencv_candidate}")
        set(_scan_tracking_opencv_lib_dir "${_scan_tracking_opencv_build_root}/x64/vc15/lib")
        set(_scan_tracking_opencv_force_cache FORCE)
    else()
        message(FATAL_ERROR
            "SCAN_TRACKING_OPENCV_DIR does not contain OpenCVConfig.cmake: "
            "${_scan_tracking_opencv_candidate}")
    endif()
else()
    set(_scan_tracking_opencv_build_root "")
    foreach(_candidate
        "${CMAKE_SOURCE_DIR}/third_party/opencv-3.4.3-vc14_vc15/opencv/build"
        "${CMAKE_SOURCE_DIR}/third_party/LB/opencv-3.4.3-vc14_vc15/opencv/build")
        if(NOT _scan_tracking_opencv_build_root AND EXISTS "${_candidate}/include")
            set(_scan_tracking_opencv_build_root "${_candidate}")
        endif()
    endforeach()
    if(NOT _scan_tracking_opencv_build_root)
        set(_scan_tracking_opencv_build_root
            "${CMAKE_SOURCE_DIR}/third_party/opencv-3.4.3-vc14_vc15/opencv/build")
    endif()
    set(_scan_tracking_opencv_lib_dir "${_scan_tracking_opencv_build_root}/x64/vc15/lib")
endif()

set(
    SCAN_TRACKING_OPENCV_BUILD_ROOT
    "${_scan_tracking_opencv_build_root}"
    CACHE PATH
    "Path to the bundled OpenCV build root"
    ${_scan_tracking_opencv_force_cache}
)
set(
    SCAN_TRACKING_OPENCV_LIB_DIR
    "${_scan_tracking_opencv_lib_dir}"
    CACHE PATH
    "Path to the bundled OpenCV import library directory"
    ${_scan_tracking_opencv_force_cache}
)
set(
    SCAN_TRACKING_OPENCV_INCLUDE_DIR
    "${_scan_tracking_opencv_build_root}/include"
    CACHE PATH
    "Path to the bundled OpenCV headers"
    ${_scan_tracking_opencv_force_cache}
)
if(_scan_tracking_opencv_force_cache OR NOT DEFINED SCAN_TRACKING_OPENCV_RUNTIME_DIR OR "${SCAN_TRACKING_OPENCV_RUNTIME_DIR}" STREQUAL "")
    set(
        SCAN_TRACKING_OPENCV_RUNTIME_DIR
        "${_scan_tracking_opencv_build_root}/x64/vc15/bin"
        CACHE PATH
        "Path to the local OpenCV runtime DLL directory"
        ${_scan_tracking_opencv_force_cache}
    )
endif()

function(scan_tracking_deploy_opencv_runtime target_name)
    if(NOT EXISTS "${SCAN_TRACKING_OPENCV_RUNTIME_DIR}")
        message(WARNING "OpenCV runtime directory not found: ${SCAN_TRACKING_OPENCV_RUNTIME_DIR}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SCAN_TRACKING_OPENCV_RUNTIME_DIR}"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMENT "Deploying OpenCV runtime"
    )
endfunction()

function(scan_tracking_deploy_lb_pose_runtime target_name)
    set(_lb_src_dir "${CMAKE_SOURCE_DIR}/third_party/LB")
    if(NOT EXISTS "${_lb_src_dir}/track_config.ini")
        message(WARNING "LB track_config.ini not found: ${_lb_src_dir}/track_config.ini")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target_name}>/third_party/LB"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_lb_src_dir}/track_config.ini"
            "$<TARGET_FILE_DIR:${target_name}>/third_party/LB/track_config.ini"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_lb_src_dir}/S2_Scanner_Template_20260717.txt"
            "$<TARGET_FILE_DIR:${target_name}>/third_party/LB/S2_Scanner_Template_20260717.txt"
        COMMENT "Deploying LB pose config and scanner template"
    )
endfunction()
