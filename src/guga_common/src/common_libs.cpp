#include "guga_common/common_libs.hpp"

double unwarp_angle(double current, const double previous)
{
    while (current - previous > M_PI)
        current -= 2.0 * M_PI;

    while (current - previous < -M_PI)
        current += 2.0 * M_PI;

    return current;
}
