#include "../src/differential_law.hpp"
#include <cmath>
#include <iostream>
int main()
{
  using rover_simulation::DifferentialTorque;
  if (DifferentialTorque(.2-.2,.3-.3,1500,60,200)!=0) return 1;
  if (DifferentialTorque(.1,0,1500,60,200)!=-150) return 2;
  if (DifferentialTorque(-.1,0,1500,60,200)!=150) return 3;
  for (double e : {-2.,-.1,0.,.1,2.})
    for (double v : {-100.,-1.,0.,1.,100.})
    {
      auto torque=DifferentialTorque(e,v,1500,60,200);
      auto spring=DifferentialTorque(e,0,1500,60,200);
      if (std::abs(torque)>200 || (torque-spring)*v>1e-10) return 4;
    }
  std::cout << "Differential symmetry, restoring torque, saturation and passivity passed\n";
  return 0;
}
