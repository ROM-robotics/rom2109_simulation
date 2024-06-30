#ifndef ROM_MATH_DOUBLE
#define ROM_MATH_DOUBLE
#pragma once

#include <iostream>
#include <cmath>

namespace rom_dynamics
{
    namespace math
    {
        bool isDoubleGreater(double a, double b, double epsilon = 1e-9) 
        {   // true if a > b, false b >= a
            return (a - b) > epsilon; 
        }
        bool areDoublesEqual(double a, double b, double epsilon = 1e-9) 
        {   
            return std::fabs(a - b) < epsilon;
        }
        bool isDoubleLesser(double a, double b, double epsilon = 1e-9) 
        {   // true if a < b, false if a >= b
            return (b - a) > epsilon;
        }
    };

};
#endif // ROM_DYNAMICS_DoubleFloat