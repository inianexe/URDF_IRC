#pragma once
#include <algorithm>
namespace rover_simulation {
inline double MotorTorque(double target,double measured,double kp,double cap)
{
  return std::clamp(kp*(target-measured),-cap,cap);
}
}
