#include "guga_common/common_libs.hpp"

double unwrap_angle(double current, const double previous)
{
    while (current - previous > M_PI)
        current -= 2.0 * M_PI;

    while (current - previous < -M_PI)
        current += 2.0 * M_PI;

    return current;
}

template <typename T>
T slidingWindowFilter(const T& current_value, std::deque<T>& window, std::size_t window_size)
{
    // 加入当前值
    window.push_back(current_value);

    // 超过窗口大小，删除最旧的数据
    if (window.size() > window_size)
    {
        window.pop_front();
    }

    // 计算窗口均值
    T sum{};

    for (const auto& value : window)
    {
        sum += value;
    }

    return sum / static_cast<T>(window.size());
}
