#ifndef GUGA_COMMON_COMMON_LIBS_HPP
#define GUGA_COMMON_COMMON_LIBS_HPP

#include <string>

/// ANSI终端颜色控制码：红色，用于错误输出
const std::string RED = "\033[1;31m";
/// ANSI终端颜色控制码：绿色，用于成功输出
const std::string GREEN = "\033[1;32m";
/// ANSI终端颜色控制码：黄色，用于警告输出
const std::string YELLOW = "\033[1;33m";
/// ANSI终端颜色控制码：蓝色，用于信息输出
const std::string BLUE = "\033[1;34m";
/// ANSI终端颜色重置码
const std::string RESET = "\033[0m";

#endif // GUGA_COMMON_COMMON_LIBS_HPP