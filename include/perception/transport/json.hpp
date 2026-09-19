#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Minimal project-owned JSON value used by the perception <-> robot-agent/payload boundary.
// It exists instead of a third-party parser because this boundary is wire-sensitive: the
// serialized lexical form of a number is part of the contract (a real value always keeps its
// decimal point) and object field order must stay deterministic for brittle peers.
namespace perception::transport::json {

class Value;

using ArrayItems = std::vector<Value>;
using ObjectMembers = std::vector<std::pair<std::string, Value>>;

class Value {
  public:
    enum class Kind { Null, Bool, Integer, Real, String, Array, Object };

    Value() = default;

    [[nodiscard]] static auto boolean(bool value) -> Value;
    [[nodiscard]] static auto integer(std::int64_t value) -> Value;
    [[nodiscard]] static auto real(double value) -> Value;
    [[nodiscard]] static auto string(std::string value) -> Value;
    [[nodiscard]] static auto array(ArrayItems value) -> Value;
    [[nodiscard]] static auto object(ObjectMembers value) -> Value;

    [[nodiscard]] auto kind() const noexcept -> Kind { return kind_; }
    [[nodiscard]] auto is_null() const noexcept -> bool { return kind_ == Kind::Null; }

    // Object access. `find` returns nullptr when this value is not an object or has no such key.
    [[nodiscard]] auto find(std::string_view key) const noexcept -> const Value*;
    void set(std::string key, Value value);

    [[nodiscard]] auto as_bool() const noexcept -> std::optional<bool>;
    [[nodiscard]] auto as_integer() const noexcept -> std::optional<std::int64_t>;
    // Accepts both Integer and Real so a peer writing `0` where `0.0` is expected still decodes.
    [[nodiscard]] auto as_number() const noexcept -> std::optional<double>;
    [[nodiscard]] auto as_string() const noexcept -> const std::string*;
    [[nodiscard]] auto as_array() const noexcept -> const ArrayItems*;

    // Objects are read by index so that field order stays observable. `std::pair` may not be
    // instantiated with an incomplete type, so members are stored as parallel key/value vectors
    // rather than as a vector of pairs inside this class.
    [[nodiscard]] auto member_count() const noexcept -> std::size_t;
    [[nodiscard]] auto key_at(std::size_t index) const noexcept -> const std::string&;
    [[nodiscard]] auto value_at(std::size_t index) const noexcept -> const Value&;

    [[nodiscard]] auto dump() const -> std::string;

  private:
    Kind kind_{Kind::Null};
    bool bool_{false};
    std::int64_t integer_{0};
    double real_{0.0};
    std::string string_;
    // Array elements, or an object's member values in declaration order.
    std::vector<Value> items_;
    // Member keys, parallel to `items_`, and empty for arrays.
    std::vector<std::string> keys_;
};

// Parses one complete JSON document. Trailing whitespace is accepted, trailing data is not.
// Returns std::nullopt and fills `error` (when non-null) instead of throwing, because this
// runs on untrusted bytes from a socket.
[[nodiscard]] auto parse(std::string_view text, std::string* error) -> std::optional<Value>;

} // namespace perception::transport::json
