#pragma once

/// Single include point for the generated MAVLink2 headers.
///
/// Two reasons for the indirection:
///  * the generated headers trigger -Waddress-of-packed-member; suppressing it
///    here keeps the project's own code compiling with full warnings instead of
///    weakening the flags globally as the example CMakeLists does;
///  * grepping for this file gives the exact list of translation units that
///    know MAVLink exists. That list must never include the domain.

#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#  pragma GCC diagnostic ignored "-Wpedantic"
#endif
#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable : 4244 4267)
#endif

#include <common/mavlink.h>

#if defined(_MSC_VER)
#  pragma warning(pop)
#endif
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#endif
