#ifndef GUGA_COMMON_COMMON_LIBS_HPP
#define GUGA_COMMON_COMMON_LIBS_HPP

#include <cmath>
#include <string>

// color
const std::string RED = "\033[1;31m";
const std::string GREEN = "\033[1;32m";
const std::string YELLOW = "\033[1;33m";
const std::string BLUE = "\033[1;34m";
const std::string PINK = "\033[1;35m";
const std::string CYAN = "\033[1;36m";
const std::string WHITE = "\033[1;37m";

// text style
const std::string RESET = "\033[0m";
const std::string BOLD = "\033[1m";
const std::string UNDERLINE = "\033[4m";

inline double unwrap_angle(double current, const double previous)
{
    while (current - previous > M_PI)
        current -= 2.0 * M_PI;

    while (current - previous < -M_PI)
        current += 2.0 * M_PI;

    return current;
}

#endif // GUGA_COMMON_COMMON_LIBS_HPP