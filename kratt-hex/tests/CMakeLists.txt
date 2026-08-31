# Two binaries on purpose: the domain suite must be runnable on its own, and it
# must link only against kratt_domain. If someone sneaks a socket into the
# hexagon, this target stops linking.
add_executable(DomainTests
    domain/test_geofence.cpp
    domain/test_kinematics.cpp)
target_link_libraries(DomainTests PRIVATE kratt_domain gtest_main kratt_warnings)

add_executable(IntegrationTests
    integration/test_disarm_safety.cpp
    integration/test_geofence.cpp
    integration/test_link.cpp)
target_include_directories(IntegrationTests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IntegrationTests PRIVATE kratt_app gtest_main kratt_warnings)

include(GoogleTest)
gtest_discover_tests(DomainTests)
gtest_discover_tests(IntegrationTests)

# Architecture fitness function: fails the build if the domain ever gains a
# forbidden dependency. Cheaper and more reliable than a code-review convention.
add_test(NAME domain_purity
         COMMAND ${CMAKE_COMMAND}
                 -DDOMAIN_DIR=${CMAKE_SOURCE_DIR}/src/domain
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/domain_purity.cmake)
