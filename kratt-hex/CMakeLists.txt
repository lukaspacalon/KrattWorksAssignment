cmake_minimum_required(VERSION 3.20)
project(KrattTestAssignment LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(KRATT_BUILD_TESTS "Build the test suites"           ON)
option(KRATT_BUILD_GUI   "Build the ImGui ground station"  ON)
option(KRATT_ENABLE_ASAN "AddressSanitizer + UBSan"        OFF)

# Warnings are attached through an interface target rather than set globally, so
# the vendored third-party code in libs/ keeps its own (looser) settings.
add_library(kratt_warnings INTERFACE)
if(MSVC)
    target_compile_options(kratt_warnings INTERFACE /W4 /permissive-)
else()
    target_compile_options(kratt_warnings INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast)
endif()

if(KRATT_ENABLE_ASAN AND NOT MSVC)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

find_package(Threads REQUIRED)

add_subdirectory(libs)
add_subdirectory(src)

if(KRATT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

message(STATUS "")
message(STATUS "KrattWorks drone - hexagonal architecture")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  tests      : ${KRATT_BUILD_TESTS}")
message(STATUS "  GUI        : ${KRATT_GUI_AVAILABLE}")
message(STATUS "  sanitizers : ${KRATT_ENABLE_ASAN}")
message(STATUS "")
