#pragma once

#include <cstdlib>
#include <iostream>

#define CHECK_TRUE(expression)                                                                    \
    do {                                                                                          \
        if (!(expression)) {                                                                      \
            std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #expression << '\n'; \
            return false;                                                                         \
        }                                                                                         \
    } while (false)
