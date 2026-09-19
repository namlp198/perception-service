#include <iostream>

auto bounded_queue_test() -> bool;
auto config_test() -> bool;
auto camera_health_test() -> bool;
auto depth_visualizer_test() -> bool;
auto transport_test() -> bool;
auto http_test() -> bool;

int main() {
    const bool passed =
        bounded_queue_test() && config_test() && camera_health_test() &&
        depth_visualizer_test() && transport_test() && http_test();
    if (!passed) {
        return 1;
    }
    std::cout << "All unit tests passed.\n";
    return 0;
}
