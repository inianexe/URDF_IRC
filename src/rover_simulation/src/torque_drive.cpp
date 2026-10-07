// Six independent output-shaft torque-limited velocity servos.
// NOT a battery/current/thermal model or a measured torque-speed envelope.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include "motor_law.hpp"

namespace rover_simulation {
class TorqueDrive final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  std::array<gz::sim::Joint,6> joints;
  const std::array<std::string,6> names={"front_left_wheel_joint","middle_left_wheel_joint",
    "rear_left_wheel_joint","front_right_wheel_joint","middle_right_wheel_joint","rear_right_wheel_joint"};
  gz::transport::Node transport;
  gz::transport::Node::Publisher diagnostics;
  std::mutex mutex;
  std::chrono::steady_clock::time_point received{};
  double desiredV{0},desiredW{0},v{0},w{0},radius{.15},track{.815};
  double cap{25},kp{5},speedLimit{12},timeout{.8},maxV{.6},maxW{.8};
  double accel{.4},angularAccel{1.2},lastPublish{-1};
  bool ready{false};
  void Command(const gz::msgs::Twist &message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const double linear=message.linear().x(),angular=message.angular().z();
    desiredV=std::isfinite(linear)?std::clamp(linear,-maxV,maxV):0;
    desiredW=std::isfinite(angular)?std::clamp(angular,-maxW,maxW):0;
    received=std::chrono::steady_clock::now();
  }
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override
  {
    radius=sdf->Get<double>("wheel_radius",radius).first;
    track=sdf->Get<double>("wheel_separation",track).first;
    cap=sdf->Get<double>("torque_limit",cap).first;
    kp=sdf->Get<double>("speed_kp",kp).first;
    speedLimit=sdf->Get<double>("speed_limit",speedLimit).first;
    timeout=sdf->Get<double>("command_timeout",timeout).first;
    maxV=sdf->Get<double>("max_linear_velocity",maxV).first;
    maxW=sdf->Get<double>("max_angular_velocity",maxW).first;
    accel=sdf->Get<double>("max_linear_acceleration",accel).first;
    angularAccel=sdf->Get<double>("max_angular_acceleration",angularAccel).first;
    for (double value:{radius,track,cap,kp,speedLimit,timeout,maxV,maxW,accel,angularAccel})
      if (!std::isfinite(value) || value<=0) {gzerr<<"Invalid torque-drive parameter\n";return;}
    gz::sim::Model model(entity);
    for (size_t i=0;i<joints.size();++i)
    {
      joints[i]=gz::sim::Joint(model.JointByName(ecm,names[i]));
      if (!joints[i].Valid(ecm)) {gzerr<<"Missing drive joint "<<names[i]<<"\n";return;}
      joints[i].EnableVelocityCheck(ecm);
    }
    diagnostics=transport.Advertise<gz::msgs::StringMsg>("/drive/motor_status");
    ready=transport.Subscribe("/cmd_vel",&TorqueDrive::Command,this);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready || info.paused)return;
    double targetV,targetW,commandAge;
    bool commandStale;
    {
      std::lock_guard<std::mutex> lock(mutex);
      commandAge=std::chrono::duration<double>(std::chrono::steady_clock::now()-received).count();
      commandStale=commandAge>timeout;
      targetV=commandStale?0:desiredV;targetW=commandStale?0:desiredW;
    }
    const double dt=std::chrono::duration<double>(info.dt).count();
    if (dt<0) {v=w=0;lastPublish=-1;return;}
    // A stop/timeout immediately requests zero wheel speed. The actual rover
    // still decelerates physically under bounded braking torque.
    if (targetV==0 && targetW==0) {v=w=0;}
    else {
      v+=std::clamp(targetV-v,-accel*dt,accel*dt);
      w+=std::clamp(targetW-w,-angularAccel*dt,angularAccel*dt);
    }
    std::array<double,6> speed{},torque{},power{};
    double motoring=0,braking=0;
    for (size_t i=0;i<joints.size();++i)
    {
      const auto measured=joints[i].Velocity(ecm);
      if (!measured || measured->empty() || !std::isfinite(measured->at(0)))
      {joints[i].SetForce(ecm,{0});continue;}
      speed[i]=measured->at(0);
      const double target=std::clamp((v+(i<3?-1:1)*w*track/2)/radius,-speedLimit,speedLimit);
      torque[i]=MotorTorque(target,speed[i],kp,cap);
      joints[i].SetForce(ecm,{torque[i]});
      power[i]=torque[i]*speed[i];
      motoring+=std::max(0.,power[i]);braking+=std::max(0.,-power[i]);
    }
    const double time=std::chrono::duration<double>(info.simTime).count();
    if (time-lastPublish<.05)return;
    lastPublish=time;
    std::ostringstream json;
    json<<"{\"sim_time\":"<<time<<",\"torque_limit_nm\":"<<cap
        <<",\"command_stale\":"<<(commandStale?"true":"false")
        <<",\"command_age_seconds\":"<<commandAge;
    json<<",\"joint_order\":[";
    for (size_t i=0;i<6;++i)json<<(i?",":"")<<'"'<<names[i]<<'"';
    json<<"]";
    auto array=[&json](const char *name,const std::array<double,6> &values) {
      json<<",\""<<name<<"\":[";
      for (size_t i=0;i<6;++i)json<<(i?",":"")<<values[i];
      json<<"]";
    };
    array("speed_rad_s",speed);array("torque_nm",torque);array("mechanical_power_w",power);
    json<<",\"motoring_power_w\":"<<motoring<<",\"braking_power_w\":"<<braking
        <<",\"battery_model\":\"unconstrained_supply_assumption\"}";
    gz::msgs::StringMsg message;message.set_data(json.str());diagnostics.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(rover_simulation::TorqueDrive,gz::sim::System,
             rover_simulation::TorqueDrive::ISystemConfigure,
             rover_simulation::TorqueDrive::ISystemPreUpdate)
