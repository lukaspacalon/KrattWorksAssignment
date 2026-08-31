# Vendored dependencies (git submodules). Each is wrapped in a target here
# instead of leaking include_directories() into the global scope: that is what
# lets the project's own code keep -Wall -Wextra without drowning in warnings
# from generated MAVLink headers.
set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT EXISTS ${LIBS_DIR}/mavlink/common/mavlink.h)
    message(FATAL_ERROR "libs/mavlink missing. Run: git submodule update --init --recursive")
endif()
add_library(mavlink INTERFACE)
# SYSTEM: warnings from generated headers are not ours to fix.
target_include_directories(mavlink SYSTEM INTERFACE ${LIBS_DIR}/mavlink)

if(KRATT_BUILD_TESTS)
    if(NOT EXISTS ${LIBS_DIR}/googletest/CMakeLists.txt)
        message(FATAL_ERROR "libs/googletest missing. Run: git submodule update --init --recursive")
    endif()
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    add_subdirectory(${LIBS_DIR}/googletest EXCLUDE_FROM_ALL)
endif()

set(KRATT_GUI_AVAILABLE OFF CACHE INTERNAL "")
if(KRATT_BUILD_GUI)
    find_package(OpenGL)
    if(NOT OpenGL_FOUND)
        message(WARNING "OpenGL not found - GCS will build headless")
    elseif(NOT EXISTS ${LIBS_DIR}/glfw/CMakeLists.txt OR NOT EXISTS ${LIBS_DIR}/imgui/imgui.cpp)
        message(WARNING "libs/glfw or libs/imgui missing - GCS will build headless")
    else()
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
        add_subdirectory(${LIBS_DIR}/glfw EXCLUDE_FROM_ALL)
        set(IMGUI_DIR ${LIBS_DIR}/imgui)
        # Explicit sources rather than file(GLOB): globbing silently pulls in
        # imgui_demo.cpp and breaks incremental builds.
        add_library(imgui STATIC
            ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp
            ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp
            ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
            ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)
        target_include_directories(imgui SYSTEM PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
        target_link_libraries(imgui PUBLIC OpenGL::GL glfw ${CMAKE_DL_LIBS})
        set(KRATT_GUI_AVAILABLE ON CACHE INTERNAL "")
    endif()
endif()
