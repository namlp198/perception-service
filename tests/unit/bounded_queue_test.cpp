#include "test_support.hpp"

#include "perception/core/bounded_queue.hpp"

auto bounded_queue_test() -> bool {
    perception::core::BoundedQueue<int> queue(2);
    queue.push(1);
    queue.push(2);
    queue.push(3);
    CHECK_TRUE(queue.size() == 2);
    CHECK_TRUE(queue.dropped_count() == 1);
    CHECK_TRUE(queue.try_pop() == 2);
    CHECK_TRUE(queue.try_pop() == 3);
    CHECK_TRUE(!queue.try_pop().has_value());
    return true;
}
