// Passive compliant gear constraint, evaluated at the physics timestep.
// Both joint axes are +Y. A mechanical differential requires qL + qR = 0
// relative to the chassis. Equal generalized torques derive from the potential
// U = k/2 * (qL+qR)^2 and dissipate d*(vL+vR)^2. Opposite articulation is free.
// This is NOT two independent springs to zero, nor a position/velocity override.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include "differential_law.hpp"

namespace rover_simulation
{
class Differential final : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
{
  gz::sim::Joint left, right;
  double stiffness{1500.}, damping{60.}, maxTorque{200.};
  bool ready{false};
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    left = gz::sim::Joint(model.JointByName(ecm,
        sdf->Get<std::string>("left_joint", "left_rocker_joint").first));
    right = gz::sim::Joint(model.JointByName(ecm,
        sdf->Get<std::string>("right_joint", "right_rocker_joint").first));
    stiffness = sdf->Get<double>("stiffness", stiffness).first;
    damping = sdf->Get<double>("damping", damping).first;
    maxTorque = sdf->Get<double>("max_torque", maxTorque).first;
    if (!left.Valid(ecm) || !right.Valid(ecm) ||
        !std::isfinite(stiffness) || !std::isfinite(damping) ||
        !std::isfinite(maxTorque) || stiffness <= 0 || damping < 0 || maxTorque <= 0)
    {
      gzerr << "[rover differential] Invalid joints or coefficients; disabled.\n";
      return;
    }
    left.EnablePositionCheck(ecm); right.EnablePositionCheck(ecm);
    left.EnableVelocityCheck(ecm); right.EnableVelocityCheck(ecm);
    ready = true;
    gzmsg << "[rover differential] Passive q_left + q_right coupling enabled; k="
          << stiffness << ", d=" << damping << ", limit=" << maxTorque << " Nm\n";
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready || info.paused) return;
    const auto ql=left.Position(ecm), qr=right.Position(ecm);
    const auto vl=left.Velocity(ecm), vr=right.Velocity(ecm);
    if (!ql || !qr || !vl || !vr || ql->empty() || qr->empty() ||
        vl->empty() || vr->empty()) return;
    const double error=ql->at(0)+qr->at(0);
    const double rate=vl->at(0)+vr->at(0);
    if (!std::isfinite(error) || !std::isfinite(rate))
    {
      left.SetForce(ecm,{0.}); right.SetForce(ecm,{0.});
      return;
    }
    // Saturated spring remains conservative; damping must remain dissipative.
    const double torque=DifferentialTorque(error,rate,stiffness,damping,maxTorque);
    left.SetForce(ecm,{torque}); right.SetForce(ecm,{torque});
  }
};
}
GZ_ADD_PLUGIN(rover_simulation::Differential, gz::sim::System,
              rover_simulation::Differential::ISystemConfigure,
              rover_simulation::Differential::ISystemPreUpdate)
