# Architecture test, executed by CTest.
#
# The hexagon's defining property is that it depends on nothing. This script
# asserts it mechanically: it greps every domain source for includes and symbols
# that would betray a leak of infrastructure into the business rules.
#
# Keeping this in the test suite rather than in a document means the constraint
# survives contact with future edits.

set(FORBIDDEN_PATTERNS
    "mavlink"          # protocol must stay in the adapter
    "sys/socket"       # transport must stay in the adapter
    "winsock"
    "arpa/inet"
    "imgui"            # presentation must stay in the adapter
    "GLFW"
    "<thread>"         # concurrency is a composition-root decision
    "<mutex>"
    "std::thread"
    "steady_clock::now"  # the domain never reads a clock; time is injected
    "system_clock::now"
    "std::cout"        # I/O goes through IEventLog
    "printf")

# Comments are stripped before scanning: the documentation legitimately names
# the things the domain must not depend on ("this is not a mavlink_message_t"),
# and a check that cannot tell prose from code is a check nobody will trust.
function(strip_comments INPUT OUTPUT_VARIABLE)
    string(REGEX REPLACE "/\\*[^*]*\\*+([^/*][^*]*\\*+)*/" "" STRIPPED "${INPUT}")
    string(REGEX REPLACE "//[^\n]*" "" STRIPPED "${STRIPPED}")
    set(${OUTPUT_VARIABLE} "${STRIPPED}" PARENT_SCOPE)
endfunction()

file(GLOB_RECURSE DOMAIN_SOURCES "${DOMAIN_DIR}/*.hpp" "${DOMAIN_DIR}/*.cpp")
if(DOMAIN_SOURCES STREQUAL "")
    message(FATAL_ERROR "no domain sources found under ${DOMAIN_DIR}")
endif()

set(VIOLATIONS "")
foreach(SOURCE ${DOMAIN_SOURCES})
    file(READ ${SOURCE} RAW_CONTENT)
    strip_comments("${RAW_CONTENT}" CONTENT)
    foreach(PATTERN ${FORBIDDEN_PATTERNS})
        string(FIND "${CONTENT}" "${PATTERN}" FOUND)
        if(NOT FOUND EQUAL -1)
            list(APPEND VIOLATIONS "${SOURCE} contains '${PATTERN}'")
        endif()
    endforeach()
endforeach()

list(LENGTH DOMAIN_SOURCES SOURCE_COUNT)
if(VIOLATIONS)
    foreach(VIOLATION ${VIOLATIONS})
        message(SEND_ERROR "domain purity violation: ${VIOLATION}")
    endforeach()
    message(FATAL_ERROR "the domain must not depend on infrastructure")
endif()
message(STATUS "domain purity OK - ${SOURCE_COUNT} files, no infrastructure dependency")
