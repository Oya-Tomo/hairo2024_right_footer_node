#include "utils.h"

double rad_to_deg(double rad)
{
    return rad * 180.0 / M_PI;
}

double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

double guard(double x, double min, double max)
{
    return (x < min) ? min : (x > max ? max : x);
}