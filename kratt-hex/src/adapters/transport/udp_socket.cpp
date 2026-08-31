#include "adapters/transport/udp_socket.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#if defined(_WIN32)
#  include <winsock2.h>
#  include <ws2tcpip.h>
using socket_t = SOCKET;
using socklen_compat = int;
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <poll.h>
#  include <sys/socket.h>
#  include <unistd.h>
using socket_t = int;
using socklen_compat = socklen_t;
static constexpr socket_t INVALID_SOCKET = -1;
#endif

namespace kratt::adapters {
namespace {

#if defined(_WIN32)
/// Winsock needs a per-process init. A function-local static gives us
/// thread-safe, once-only initialisation with no explicit call site.
void ensure_initialised() {
    struct Guard {
        Guard() {
            WSADATA data{};
            if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
                throw std::runtime_error{"WSAStartup failed"};
            }
        }
        ~Guard() { WSACleanup(); }
    };
    static Guard guard;
}
int last_error() { return WSAGetLastError(); }
void close_socket(socket_t s) { closesocket(s); }
#else
void ensure_initialised() {}
int last_error() { return errno; }
void close_socket(socket_t s) { ::close(s); }
#endif

sockaddr_in to_sockaddr(const Endpoint& endpoint) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(endpoint.port);
    if (::inet_pton(AF_INET, endpoint.address.c_str(), &addr.sin_addr) != 1) {
        throw std::runtime_error{"invalid IPv4 address: " + endpoint.address};
    }
    return addr;
}

Endpoint from_sockaddr(const sockaddr_in& addr) {
    char text[INET_ADDRSTRLEN]{};
    ::inet_ntop(AF_INET, &addr.sin_addr, text, sizeof(text));
    return Endpoint{text, ntohs(addr.sin_port)};
}

}  // namespace

UdpSocket::UdpSocket(const Endpoint& bind_address) {
    ensure_initialised();

    const socket_t fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd == INVALID_SOCKET) {
        throw std::runtime_error{"socket() failed, errno=" + std::to_string(last_error())};
    }
    fd_ = static_cast<std::intptr_t>(fd);

    const sockaddr_in addr = to_sockaddr(bind_address);
    if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        const int error = last_error();
        close_socket(fd);
        fd_ = -1;
        throw std::runtime_error{"bind(" + bind_address.to_string() +
                                 ") failed, errno=" + std::to_string(error)};
    }

    sockaddr_in bound{};
    socklen_compat length = sizeof(bound);
    local_ = (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &length) == 0)
                 ? from_sockaddr(bound)
                 : bind_address;
}

UdpSocket::~UdpSocket() {
    if (fd_ >= 0) {
        close_socket(static_cast<socket_t>(fd_));
        fd_ = -1;
    }
}

bool UdpSocket::send(std::span<const std::uint8_t> payload, const Endpoint& to) {
    if (fd_ < 0 || payload.empty()) {
        return false;
    }
    const sockaddr_in addr = to_sockaddr(to);
    const auto sent = ::sendto(static_cast<socket_t>(fd_),
                               reinterpret_cast<const char*>(payload.data()),
                               static_cast<int>(payload.size()), 0,
                               reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    return sent >= 0 && static_cast<std::size_t>(sent) == payload.size();
}

std::optional<Datagram> UdpSocket::receive(std::chrono::milliseconds timeout) {
    if (fd_ < 0 || shutting_down_.load(std::memory_order_acquire)) {
        return std::nullopt;
    }

    // Readiness is checked first so the call always returns within `timeout`
    // and the worker thread stays responsive to a shutdown request.
#if defined(_WIN32)
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(static_cast<socket_t>(fd_), &read_set);
    timeval tv{};
    tv.tv_sec = static_cast<long>(timeout.count() / 1000);
    tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
    const int ready = ::select(0, &read_set, nullptr, nullptr, &tv);
#else
    pollfd poll_fd{};
    poll_fd.fd = static_cast<int>(fd_);
    poll_fd.events = POLLIN;
    const int ready = ::poll(&poll_fd, 1, static_cast<int>(timeout.count()));
#endif
    if (ready <= 0) {
        return std::nullopt;  // timeout or EINTR: the caller simply retries
    }

    sockaddr_in from{};
    socklen_compat from_length = sizeof(from);
    const auto received = ::recvfrom(static_cast<socket_t>(fd_),
                                     reinterpret_cast<char*>(buffer_.data()),
                                     static_cast<int>(buffer_.size()), 0,
                                     reinterpret_cast<sockaddr*>(&from), &from_length);
    if (received <= 0) {
        return std::nullopt;
    }

    Datagram datagram;
    datagram.payload.assign(buffer_.begin(), buffer_.begin() + received);
    datagram.from = from_sockaddr(from);
    return datagram;
}

void UdpSocket::shutdown() {
    shutting_down_.store(true, std::memory_order_release);
    // A one-byte datagram to ourselves wakes the pending poll()/select()
    // immediately instead of waiting for the timeout to elapse.
    if (fd_ >= 0) {
        const std::uint8_t byte{0};
        const Endpoint self = (local_.address == "0.0.0.0")
                                  ? Endpoint{"127.0.0.1", local_.port}
                                  : local_;
        const sockaddr_in addr = to_sockaddr(self);
        ::sendto(static_cast<socket_t>(fd_), reinterpret_cast<const char*>(&byte), 1, 0,
                 reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    }
}

}  // namespace kratt::adapters
