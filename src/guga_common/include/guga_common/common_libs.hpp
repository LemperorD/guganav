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

const std::string RED_LIGHT = "\033[1;91m";
const std::string GREEN_LIGHT = "\033[1;92m";
const std::string YELLOW_LIGHT = "\033[1;93m";
const std::string BLUE_LIGHT = "\033[1;94m";
const std::string PINK_LIGHT = "\033[1;95m";
const std::string CYAN_LIGHT = "\033[1;96m";
const std::string WHITE_LIGHT = "\033[1;97m";

// text style
const std::string RESET = "\033[0m";
const std::string BOLD = "\033[1m";
const std::string UNDERLINE = "\033[4m";

double unwrap_angle(double current, const double previous);

#endif // GUGA_COMMON_COMMON_LIBS_HPP