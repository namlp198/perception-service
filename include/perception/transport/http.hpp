#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

// The robot-agent side of this boundary speaks HTTP/1.1 JSON (it is the same server the direct
// control UI uses), while payload-service speaks bare JSON over TCP. Only the minimum needed to
// issue one GET and read one bounded response lives here, and it is kept free of sockets so the
// framing rules stay unit-testable on a host.
namespace perception::transport::http {

[[nodiscard]] auto build_get_request(std::string_view host, std::uint16_t port,
                                    std::string_view path) -> std::string;

struct Response {
    int status_code{};
    std::string body;
};

// Parses a complete response. Uses `Content-Length` when present and otherwise treats everything
// after the header block as the body, which is what `Connection: close` responses give us.
// Returns nullopt with `error` filled when the response is malformed or truncated.
[[nodiscard]] auto parse_response(std::string_view raw, std::string* error)
    -> std::optional<Response>;

} // namespace perception::transport::http
