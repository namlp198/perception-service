#include "perception/transport/http.hpp"

#include "test_support.hpp"

#include <string>

namespace {

namespace http = perception::transport::http;

auto get_request_declares_host_and_closes_the_connection() -> bool {
    const std::string request = http::build_get_request("192.168.1.206", 5080, "/api/v1/status");
    CHECK_TRUE(request.rfind("GET /api/v1/status HTTP/1.1\r\n", 0) == 0);
    CHECK_TRUE(request.find("Host: 192.168.1.206:5080\r\n") != std::string::npos);
    // One request per connection on this boundary, matching how payload-service is called.
    CHECK_TRUE(request.find("Connection: close\r\n") != std::string::npos);
    CHECK_TRUE(request.size() >= 4 && request.compare(request.size() - 4, 4, "\r\n\r\n") == 0);
    return true;
}

auto response_body_respects_content_length() -> bool {
    std::string error;
    const std::string raw =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 13\r\n"
        "Connection: close\r\n\r\n{\"status\":1}\r\nTRAILING GARBAGE";
    const auto parsed = http::parse_response(raw, &error);
    CHECK_TRUE(parsed.has_value());
    CHECK_TRUE(parsed->status_code == 200);
    CHECK_TRUE(parsed->body == "{\"status\":1}\r");
    return true;
}

auto response_body_falls_back_to_connection_close() -> bool {
    std::string error;
    const std::string raw = "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\n{\"streams\":{}}";
    const auto parsed = http::parse_response(raw, &error);
    CHECK_TRUE(parsed.has_value());
    CHECK_TRUE(parsed->body == "{\"streams\":{}}");
    return true;
}

auto response_reports_error_status_without_inventing_a_body() -> bool {
    std::string error;
    const std::string raw =
        "HTTP/1.1 500 Internal Server Error\r\nContent-Length: 2\r\n\r\n{}";
    const auto parsed = http::parse_response(raw, &error);
    CHECK_TRUE(parsed.has_value());
    CHECK_TRUE(parsed->status_code == 500);
    CHECK_TRUE(parsed->body == "{}");
    return true;
}

auto truncated_and_malformed_responses_are_rejected() -> bool {
    std::string error;
    // A body shorter than its declared length is a truncated read, not a valid measurement.
    CHECK_TRUE(!http::parse_response("HTTP/1.1 200 OK\r\nContent-Length: 40\r\n\r\n{\"a\":1}",
                                     &error)
                    .has_value());
    CHECK_TRUE(!error.empty());
    CHECK_TRUE(!http::parse_response("HTTP/1.1 200 OK\r\nContent-Length: 4", &error).has_value());
    CHECK_TRUE(!http::parse_response("garbage\r\n\r\nbody", &error).has_value());
    CHECK_TRUE(!http::parse_response("HTTP/1.1 OK\r\n\r\nbody", &error).has_value());
    // Chunked bodies are never produced by this peer and must not be mis-read as a body.
    CHECK_TRUE(!http::parse_response(
                    "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n2\r\n{}\r\n0\r\n\r\n",
                    &error)
                    .has_value());
    return true;
}

auto header_lookup_ignores_case_and_surrounding_space() -> bool {
    std::string error;
    const std::string raw = "HTTP/1.1 200 OK\r\ncontent-length:   5  \r\n\r\nhello world";
    const auto parsed = http::parse_response(raw, &error);
    CHECK_TRUE(parsed.has_value());
    CHECK_TRUE(parsed->body == "hello");
    return true;
}

} // namespace

auto http_test() -> bool {
    return get_request_declares_host_and_closes_the_connection() &&
           response_body_respects_content_length() &&
           response_body_falls_back_to_connection_close() &&
           response_reports_error_status_without_inventing_a_body() &&
           truncated_and_malformed_responses_are_rejected() &&
           header_lookup_ignores_case_and_surrounding_space();
}
