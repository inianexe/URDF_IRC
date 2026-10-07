#include "../src/motor_law.hpp"
#include <cmath>
int main()
{
  using rover_simulation::MotorTorque;
  if (MotorTorque(100,0,5,25)!=25)return 1;
  if (MotorTorque(-100,0,5,25)!=-25)return 2;
  if (MotorTorque(2,2,5,25)!=0)return 3;
  if (MotorTorque(0,2,5,25)!=-10)return 4;
  for (double target:{-12.,-4.,0.,4.,12.})
    for (double actual:{-100.,-10.,0.,10.,100.})
      if (std::abs(MotorTorque(target,actual,5,25))>25)return 5;
  return 0;
}
