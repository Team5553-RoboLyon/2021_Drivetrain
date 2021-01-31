#include <iostream>


double Deadband(double value, double deadband = 0.1)
{
    if (std::abs(value) < deadband)
        return 0;
    else
        return value < 0 ? (value + deadband) / (1.0 - deadband) : (value - deadband) / (1.0 - deadband);
}
