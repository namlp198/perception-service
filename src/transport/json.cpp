#include "perception/transport/json.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace perception::transport::json {
namespace {

constexpr int kMaxDepth = 32;

void append_escaped(const std::string& text, std::string& out) {
    out.push_back('"');
    for (const char character : text) {
        switch (character) {
        case '"':
            out.append("\\\"");
            break;
        case '\\':
            out.append("\\\\");
            break;
        case '\b':
            out.append("\\b");
            break;
        case '\f':
            out.append("\\f");
            break;
        case '\n':
            out.append("\\n");
            break;
        case '\r':
            out.append("\\r");
            break;
        case '\t':
            out.append("\\t");
            break;
        default: {
            const auto code = static_cast<unsigned char>(character);
            if (code < 0x20U) {
                std::array<char, 7> buffer{};
                std::snprintf(buffer.data(), buffer.size(), "\\u%04x", static_cast<unsigned>(code));
                out.append(buffer.data());
            } else {
                out.push_back(character);
            }
            break;
        }
        }
    }
    out.push_back('"');
}

// Shortest representation that still round-trips, then forced into real form. A real value must
// never serialize as a bare integer at this boundary: a peer reading `0` where `0.0` was expected
// has already caused one field failure in this repository.
auto format_real(double value) -> std::string {
    if (!std::isfinite(value)) {
        return "null";
    }
    std::array<char, 64> buffer{};
    for (int precision = 1; precision <= 17; ++precision) {
        std::snprintf(buffer.data(), buffer.size(), "%.*g", precision, value);
        if (std::strtod(buffer.data(), nullptr) == value) {
            break;
        }
    }
    std::string text(buffer.data());
    if (text.find_first_of(".eE") == std::string::npos) {
        text.append(".0");
    }
    return text;
}

void dump_value(const Value& value, std::string& out) {
    switch (value.kind()) {
    case Value::Kind::Null:
        out.append("null");
        break;
    case Value::Kind::Bool:
        out.append(*value.as_bool() ? "true" : "false");
        break;
    case Value::Kind::Integer:
        out.append(std::to_string(*value.as_integer()));
        break;
    case Value::Kind::Real:
        out.append(format_real(*value.as_number()));
        break;
    case Value::Kind::String:
        append_escaped(*value.as_string(), out);
        break;
    case Value::Kind::Array: {
        out.push_back('[');
        bool first = true;
        for (const Value& item : *value.as_array()) {
            if (!first) {
                out.push_back(',');
            }
            first = false;
            dump_value(item, out);
        }
        out.push_back(']');
        break;
    }
    case Value::Kind::Object: {
        out.push_back('{');
        for (std::size_t index = 0; index < value.member_count(); ++index) {
            if (index != 0) {
                out.push_back(',');
            }
            append_escaped(value.key_at(index), out);
            out.push_back(':');
            dump_value(value.value_at(index), out);
        }
        out.push_back('}');
        break;
    }
    }
}

class Parser {
  public:
    Parser(std::string_view text, std::string* error) : text_(text), error_(error) {}

    auto run() -> std::optional<Value> {
        auto value = parse_value(0);
        if (!value) {
            return std::nullopt;
        }
        skip_whitespace();
        if (index_ != text_.size()) {
            return fail("trailing data after JSON document");
        }
        return value;
    }

  private:
    auto fail(const char* message) -> std::optional<Value> {
        if (error_ != nullptr && error_->empty()) {
            *error_ = message;
        }
        return std::nullopt;
    }

    void skip_whitespace() {
        while (index_ < text_.size()) {
            const char character = text_[index_];
            if (character != ' ' && character != '\t' && character != '\n' && character != '\r') {
                break;
            }
            ++index_;
        }
    }

    [[nodiscard]] auto peek() const -> char {
        return index_ < text_.size() ? text_[index_] : '\0';
    }

    auto consume(std::string_view literal) -> bool {
        if (text_.substr(index_, literal.size()) != literal) {
            return false;
        }
        index_ += literal.size();
        return true;
    }

    auto parse_value(int depth) -> std::optional<Value> {
        if (depth > kMaxDepth) {
            return fail("JSON nesting is too deep");
        }
        skip_whitespace();
        if (index_ >= text_.size()) {
            return fail("unexpected end of JSON document");
        }
        switch (peek()) {
        case 'n':
            return consume("null") ? std::optional<Value>(Value{}) : fail("invalid literal");
        case 't':
            return consume("true") ? std::optional<Value>(Value::boolean(true))
                                   : fail("invalid literal");
        case 'f':
            return consume("false") ? std::optional<Value>(Value::boolean(false))
                                    : fail("invalid literal");
        case '"': {
            auto text = parse_string();
            if (!text) {
                return std::nullopt;
            }
            return Value::string(std::move(*text));
        }
        case '[':
            return parse_array(depth);
        case '{':
            return parse_object(depth);
        default:
            return parse_number();
        }
    }

    auto parse_string() -> std::optional<std::string> {
        ++index_; // opening quote
        std::string out;
        while (index_ < text_.size()) {
            const char character = text_[index_++];
            if (character == '"') {
                return out;
            }
            if (character != '\\') {
                out.push_back(character);
                continue;
            }
            if (index_ >= text_.size()) {
                break;
            }
            const char escape = text_[index_++];
            switch (escape) {
            case '"':
                out.push_back('"');
                break;
            case '\\':
                out.push_back('\\');
                break;
            case '/':
                out.push_back('/');
                break;
            case 'b':
                out.push_back('\b');
                break;
            case 'f':
                out.push_back('\f');
                break;
            case 'n':
                out.push_back('\n');
                break;
            case 'r':
                out.push_back('\r');
                break;
            case 't':
                out.push_back('\t');
                break;
            case 'u': {
                auto code = parse_hex4();
                if (!code) {
                    return std::nullopt;
                }
                append_utf8(*code, out);
                break;
            }
            default:
                fail("invalid escape sequence");
                return std::nullopt;
            }
        }
        fail("unterminated string");
        return std::nullopt;
    }

    auto parse_hex4() -> std::optional<unsigned> {
        if (index_ + 4 > text_.size()) {
            fail("truncated unicode escape");
            return std::nullopt;
        }
        unsigned code = 0;
        for (std::size_t offset = 0; offset < 4; ++offset) {
            const char digit = text_[index_ + offset];
            unsigned nibble = 0;
            if (digit >= '0' && digit <= '9') {
                nibble = static_cast<unsigned>(digit - '0');
            } else if (digit >= 'a' && digit <= 'f') {
                nibble = static_cast<unsigned>(digit - 'a') + 10U;
            } else if (digit >= 'A' && digit <= 'F') {
                nibble = static_cast<unsigned>(digit - 'A') + 10U;
            } else {
                fail("invalid unicode escape");
                return std::nullopt;
            }
            code = (code << 4U) | nibble;
        }
        index_ += 4;
        return code;
    }

    // Lone surrogates become the replacement character rather than a hard parse failure: this
    // boundary must survive odd peer text without dropping an otherwise valid measurement.
    static void append_utf8(unsigned code, std::string& out) {
        if (code >= 0xD800U && code <= 0xDFFFU) {
            code = 0xFFFDU;
        }
        if (code < 0x80U) {
            out.push_back(static_cast<char>(code));
        } else if (code < 0x800U) {
            out.push_back(static_cast<char>(0xC0U | (code >> 6U)));
            out.push_back(static_cast<char>(0x80U | (code & 0x3FU)));
        } else {
            out.push_back(static_cast<char>(0xE0U | (code >> 12U)));
            out.push_back(static_cast<char>(0x80U | ((code >> 6U) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | (code & 0x3FU)));
        }
    }

    auto parse_number() -> std::optional<Value> {
        const std::size_t start = index_;
        if (peek() == '-' || peek() == '+') {
            ++index_;
        }
        bool is_real = false;
        while (index_ < text_.size()) {
            const char character = text_[index_];
            if (character >= '0' && character <= '9') {
                ++index_;
                continue;
            }
            if (character == '.' || character == 'e' || character == 'E') {
                is_real = true;
                ++index_;
                continue;
            }
            if ((character == '-' || character == '+') && index_ > start &&
                (text_[index_ - 1] == 'e' || text_[index_ - 1] == 'E')) {
                ++index_;
                continue;
            }
            break;
        }
        if (index_ == start) {
            return fail("unexpected character");
        }
        const std::string token(text_.substr(start, index_ - start));
        char* end = nullptr;
        if (!is_real) {
            const long long parsed = std::strtoll(token.c_str(), &end, 10);
            if (end != nullptr && *end == '\0') {
                return Value::integer(static_cast<std::int64_t>(parsed));
            }
            end = nullptr;
        }
        const double parsed = std::strtod(token.c_str(), &end);
        if (end == nullptr || *end != '\0') {
            return fail("invalid number");
        }
        return Value::real(parsed);
    }

    auto parse_array(int depth) -> std::optional<Value> {
        ++index_; // '['
        ArrayItems items;
        skip_whitespace();
        if (peek() == ']') {
            ++index_;
            return Value::array(std::move(items));
        }
        while (true) {
            auto item = parse_value(depth + 1);
            if (!item) {
                return std::nullopt;
            }
            items.push_back(std::move(*item));
            skip_whitespace();
            if (peek() == ',') {
                ++index_;
                continue;
            }
            if (peek() == ']') {
                ++index_;
                return Value::array(std::move(items));
            }
            return fail("expected ',' or ']' in array");
        }
    }

    auto parse_object(int depth) -> std::optional<Value> {
        ++index_; // '{'
        ObjectMembers members;
        skip_whitespace();
        if (peek() == '}') {
            ++index_;
            return Value::object(std::move(members));
        }
        while (true) {
            skip_whitespace();
            if (peek() != '"') {
                return fail("expected object key");
            }
            auto key = parse_string();
            if (!key) {
                return std::nullopt;
            }
            skip_whitespace();
            if (peek() != ':') {
                return fail("expected ':' after object key");
            }
            ++index_;
            auto item = parse_value(depth + 1);
            if (!item) {
                return std::nullopt;
            }
            members.emplace_back(std::move(*key), std::move(*item));
            skip_whitespace();
            if (peek() == ',') {
                ++index_;
                continue;
            }
            if (peek() == '}') {
                ++index_;
                return Value::object(std::move(members));
            }
            return fail("expected ',' or '}' in object");
        }
    }

    std::string_view text_;
    std::string* error_;
    std::size_t index_{0};
};

} // namespace

auto Value::boolean(bool value) -> Value {
    Value result;
    result.kind_ = Kind::Bool;
    result.bool_ = value;
    return result;
}

auto Value::integer(std::int64_t value) -> Value {
    Value result;
    result.kind_ = Kind::Integer;
    result.integer_ = value;
    return result;
}

auto Value::real(double value) -> Value {
    Value result;
    result.kind_ = Kind::Real;
    result.real_ = value;
    return result;
}

auto Value::string(std::string value) -> Value {
    Value result;
    result.kind_ = Kind::String;
    result.string_ = std::move(value);
    return result;
}

auto Value::array(ArrayItems value) -> Value {
    Value result;
    result.kind_ = Kind::Array;
    result.items_ = std::move(value);
    return result;
}

auto Value::object(ObjectMembers value) -> Value {
    Value result;
    result.kind_ = Kind::Object;
    result.items_.reserve(value.size());
    result.keys_.reserve(value.size());
    for (auto& member : value) {
        result.keys_.push_back(std::move(member.first));
        result.items_.push_back(std::move(member.second));
    }
    return result;
}

auto Value::find(std::string_view key) const noexcept -> const Value* {
    if (kind_ != Kind::Object) {
        return nullptr;
    }
    const auto found = std::find(keys_.begin(), keys_.end(), key);
    if (found == keys_.end()) {
        return nullptr;
    }
    return &items_[static_cast<std::size_t>(found - keys_.begin())];
}

void Value::set(std::string key, Value value) {
    if (kind_ != Kind::Object) {
        kind_ = Kind::Object;
        items_.clear();
        keys_.clear();
    }
    const auto found = std::find(keys_.begin(), keys_.end(), key);
    if (found == keys_.end()) {
        keys_.push_back(std::move(key));
        items_.push_back(std::move(value));
    } else {
        items_[static_cast<std::size_t>(found - keys_.begin())] = std::move(value);
    }
}

auto Value::as_bool() const noexcept -> std::optional<bool> {
    if (kind_ != Kind::Bool) {
        return std::nullopt;
    }
    return bool_;
}

auto Value::as_integer() const noexcept -> std::optional<std::int64_t> {
    if (kind_ != Kind::Integer) {
        return std::nullopt;
    }
    return integer_;
}

auto Value::as_number() const noexcept -> std::optional<double> {
    if (kind_ == Kind::Integer) {
        return static_cast<double>(integer_);
    }
    if (kind_ == Kind::Real) {
        return real_;
    }
    return std::nullopt;
}

auto Value::as_string() const noexcept -> const std::string* {
    return kind_ == Kind::String ? &string_ : nullptr;
}

auto Value::as_array() const noexcept -> const ArrayItems* {
    return kind_ == Kind::Array ? &items_ : nullptr;
}

auto Value::member_count() const noexcept -> std::size_t {
    return kind_ == Kind::Object ? keys_.size() : 0U;
}

auto Value::key_at(std::size_t index) const noexcept -> const std::string& { return keys_[index]; }

auto Value::value_at(std::size_t index) const noexcept -> const Value& { return items_[index]; }

auto Value::dump() const -> std::string {
    std::string out;
    dump_value(*this, out);
    return out;
}

auto parse(std::string_view text, std::string* error) -> std::optional<Value> {
    if (error != nullptr) {
        error->clear();
    }
    Parser parser(text, error);
    auto value = parser.run();
    if (!value && error != nullptr && error->empty()) {
        *error = "invalid JSON document";
    }
    return value;
}

} // namespace perception::transport::json
