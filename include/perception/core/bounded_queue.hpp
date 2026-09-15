#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace perception::core {

// A fresh-data queue: once full, the oldest item is discarded.
template <typename T> class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {
        if (capacity == 0) {
            throw std::invalid_argument("queue capacity must be greater than zero");
        }
    }

    BoundedQueue(const BoundedQueue&) = delete;
    auto operator=(const BoundedQueue&) -> BoundedQueue& = delete;

    void push(T item) {
        {
            std::scoped_lock lock(mutex_);
            if (stopped_) {
                return;
            }
            if (items_.size() == capacity_) {
                items_.pop_front();
                ++dropped_count_;
            }
            items_.push_back(std::move(item));
        }
        condition_.notify_one();
    }

    [[nodiscard]] auto try_pop() -> std::optional<T> {
        std::scoped_lock lock(mutex_);
        if (items_.empty()) {
            return std::nullopt;
        }
        T item = std::move(items_.front());
        items_.pop_front();
        return item;
    }

    void stop() {
        {
            std::scoped_lock lock(mutex_);
            stopped_ = true;
        }
        condition_.notify_all();
    }

    [[nodiscard]] auto size() const -> std::size_t {
        std::scoped_lock lock(mutex_);
        return items_.size();
    }

    [[nodiscard]] auto dropped_count() const -> std::size_t {
        std::scoped_lock lock(mutex_);
        return dropped_count_;
    }

private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<T> items_;
    std::size_t dropped_count_{0};
    bool stopped_{false};
};

}  // namespace perception::core
