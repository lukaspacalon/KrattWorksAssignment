# ---------------------------------------------------------------------------
# kratt_domain - the hexagon.
#
# Links against NOTHING. No mavlink, no sockets, no threads, no GUI. That empty
# target_link_libraries line is the architecture, expressed in the build system:
# a dependency added here would be a dependency added to the business rules, and
# CMake would be the first to say so.
# ---------------------------------------------------------------------------
add_library(kratt_domain STATIC
    domain/flight_controller.cpp
    domain/flight_state_machine.cpp
    domain/drone_service.cpp
    domain/gcs_service.cpp)
target_include_directories(kratt_domain PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kratt_domain PRIVATE kratt_warnings)

# ---------------------------------------------------------------------------
# kratt_adapters - everything the domain refuses to know about.
# ---------------------------------------------------------------------------
add_library(kratt_adapters STATIC
    adapters/transport/udp_socket.cpp
    adapters/transport/loopback_transport.cpp
    adapters/mavlink/mavlink_codec.cpp
    adapters/mavlink/message_channel.cpp
    adapters/mavlink/threaded_channel.cpp
    adapters/mavlink/drone_mavlink_adapter.cpp
    adapters/mavlink/gcs_mavlink_adapter.cpp)
target_include_directories(kratt_adapters PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kratt_adapters
    PUBLIC kratt_domain mavlink Threads::Threads
    PRIVATE kratt_warnings)
if(WIN32)
    target_link_libraries(kratt_adapters PUBLIC ws2_32)
endif()

# ---------------------------------------------------------------------------
# kratt_app - composition roots. The only place that sees both sides.
# ---------------------------------------------------------------------------
add_library(kratt_app STATIC
    app/drone_composition.cpp
    app/gcs_composition.cpp)
target_include_directories(kratt_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kratt_app PUBLIC kratt_adapters PRIVATE kratt_warnings)

add_executable(Drone bin/drone_main.cpp)
target_link_libraries(Drone PRIVATE kratt_app kratt_warnings)

add_executable(GCS bin/gcs_main.cpp)
target_link_libraries(GCS PRIVATE kratt_app kratt_warnings)
if(KRATT_GUI_AVAILABLE)
    target_sources(GCS PRIVATE bin/gcs_gui.cpp)
    target_link_libraries(GCS PRIVATE imgui)
    target_compile_definitions(GCS PRIVATE KRATT_WITH_GUI)
endif()
