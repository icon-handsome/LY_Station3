include_guard(GLOBAL)

option(SCAN_TRACKING_ENABLE_WARNINGS "Enable stronger compiler warnings" ON)

if(SCAN_TRACKING_ENABLE_WARNINGS)
    if(MSVC)
        add_compile_options(/W4 /permissive-)
    else()
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
