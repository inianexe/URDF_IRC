#pragma once
#include <algorithm>
namespace rover_simulation {
inline double DifferentialTorque(double error, double rate,
                                 double stiffness, double damping, double limit)
{
  const double spring=std::clamp(-stiffness*error,-limit,limit);
  return std::clamp(spring-damping*rate,-limit,limit);
}
}
