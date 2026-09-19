#include "perception/transport/http.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace perception::transport::http {
namespace {

auto to_lower_ascii(std::string_view text) -> std::string {
    std::string lowered(text);
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return lowered;
}

void fail(std::string* error, const char* message) {
    if (error != nullptr) {
        *error = message;
    }
}

// Returns the value of `name` from the header block, lower-cased comparison, trimmed value.
auto find_header(std::string_view headers, std::string_view name) -> std::optional<std::string> {
    const std::string lowered_headers = to_lower_ascii(headers);
    const std::string lowered_name = to_lower_ascii(name) + ":";
    std::size_t search_from = 0;
    while (search_from < lowered_headers.size()) {
        const std::size_t line_end = lowered_headers.find("\r\n", search_from);
        const std::size_t line_length = line_end == std::string::npos
                                            ? lowered_headers.size() - search_from
                                            : line_end - search_from;
        const std::string_view lowered_line(lowered_headers.data() + search_from, line_length);
        if (lowered_line.rfind(lowered_name, 0) == 0) {
            std::string_view value =
                std::string_view(headers.data() + search_from, line_length).substr(
                    lowered_name.size());
            while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
                value.remove_prefix(1);
            }
            while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
                value.remove_suffix(1);
            }
            return std::string(value);
        }
        if (line_end == std::string::npos) {
            break;
        }
        search_from = line_end + 2;
    }
    return std::nullopt;
}

} // namespace

auto build_get_request(std::string_view host, std::uint16_t port, std::string_view path)
    -> std::string {
    std::string request;
    request.append("GET ").append(path).append(" HTTP/1.1\r\n");
    request.append("Host: ").append(host).append(":").append(std::to_string(port)).append("\r\n");
    request.append("User-Agent: perception-service\r\n");
    request.append("Accept: application/json\r\n");
    // This boundary issues one request per connection, exactly as it does towards payload-service,
    // so the peer is told to close rather than leaving a half-used keep-alive socket behind.
    request.append("Connection: close\r\n\r\n");
    return request;
}

auto parse_response(std::string_view raw, std::string* error) -> std::optional<Response> {
    if (error != nullptr) {
        error->clear();
    }
    const std::size_t header_end = raw.find("\r\n\r\n");
    if (header_end == std::string_view::npos) {
        fail(error, "HTTP response has no header terminator");
        return std::nullopt;
    }

    const std::string_view head = raw.substr(0, header_end);
    const std::size_t status_line_end = head.find("\r\n");
    const std::string_view status_line = head.substr(0, status_line_end);
    if (status_line.rfind("HTTP/1.", 0) != 0) {
        fail(error, "HTTP response has no status line");
        return std::nullopt;
    }
    const std::size_t code_start = status_line.find(' ');
    if (code_start == std::string_view::npos || code_start + 4 > status_line.size()) {
        fail(error, "HTTP status line carries no status code");
        return std::nullopt;
    }
    const std::string code_text(status_line.substr(code_start + 1, 3));
    char* code_end = nullptr;
    const long code = std::strtol(code_text.c_str(), &code_end, 10);
    if (code_end == nullptr || *code_end != '\0' || code < 100 || code > 599) {
        fail(error, "HTTP status code is not a number");
        return std::nullopt;
    }

    const std::string_view headers =
        status_line_end == std::string_view::npos ? std::string_view{}
                                                 : head.substr(status_line_end + 2);
    std::string_view body = raw.substr(header_end + 4);

    // Chunked bodies are not produced by this peer and are not silently mis-read as a body.
    if (const auto encoding = find_header(headers, "Transfer-Encoding");
        encoding.has_value() && to_lower_ascii(*encoding).find("chunked") != std::string::npos) {
        fail(error, "chunked HTTP responses are not supported on this boundary");
        return std::nullopt;
    }

    if (const auto length_text = find_header(headers, "Content-Length"); length_text.has_value()) {
        char* length_end = nullptr;
        const long long declared = std::strtoll(length_text->c_str(), &length_end, 10);
        if (length_end == nullptr || *length_end != '\0' || declared < 0) {
            fail(error, "Content-Length is not a number");
            return std::nullopt;
        }
        const auto expected = static_cast<std::size_t>(declared);
        if (body.size() < expected) {
            fail(error, "HTTP body is shorter than Content-Length");
            return std::nullopt;
        }
        body = body.substr(0, expected);
    }

    Response response;
    response.status_code = static_cast<int>(code);
    response.body = std::string(body);
    return response;
}

} // namespace perception::transport::http
