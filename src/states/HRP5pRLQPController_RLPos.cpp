#include "HRP5pRLQPController_RLPos.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_RLPos::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_RLPos::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.activateTorqueControl(false);
  ctl.activateFloatingBaseReal(true);
  ctl.activateContactConstraints(false);
  ctl.activateExternalTorqueComputation(true);
  ctl.solver().addTask(ctl.torqueJointTask);
  // ctl.solver().addTask(ctl.torqueTask);
  if(ctl.postureTask->inSolver()) ctl.solver().removeTask(ctl.postureTask);
  ctl.setHighPDGains(false); // Use RL gains
  ctl.utilsClass.start_rl_state(ctl, "RL_State");

  // Reset passivity observer/controller state, one per actuated joint
  // ctl.passivityState.assign(size_t(ctl.jointNames.size()), HRP5pRLQPController::JointPassivityState{});

  // auto & robot = ctl.robots().robot(ctl.robot().robotIndex());
  // ctl.passivityDampingMax = Eigen::VectorXd::Zero(ctl.jointNames.size());
  // const double alpha = 0.25;
  // const double velRelTypical = 2.0; // rad/s, refine from logs once you have vibration data
  // for(int i = 0; i < int(ctl.jointNames.size()); ++i)
  // {
  //   if(!robot.hasJoint(ctl.jointNames[size_t(i)])) continue;
  //   const size_t mbcIndex = robot.jointIndexByName(ctl.jointNames[size_t(i)]);
  //   if(robot.tu()[mbcIndex].empty()) continue;
  //   double tauMax = robot.tu()[mbcIndex][0]; // assumes symmetric limits; use min(|tu|,|tl|) if not
  //   ctl.passivityDampingMax[size_t(i)] = alpha * tauMax / velRelTypical;
  // }
}

bool HRP5pRLQPController_RLPos::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  // auto & robot = ctl.robots().robot(ctl.robot().robotIndex());
  // auto & real_robot = ctl.realRobot(robot.name());
  // const auto & q_mbc = robot.mbc().q;
  // const auto & q_dot_mbc = robot.mbc().alpha;
  // const auto & q_dot_real_mbc = real_robot.mbc().alpha;

  // const double dt = ctl.timeStep;
  // constexpr double relVelEps = 1e-4; // guard against division blow-up near zero relative velocity

  // std::map<std::string, std::vector<double>> torqueTarget;
  // for(int i = 0; i < int(ctl.jointNames.size()); ++i)
  // {
  //   if(!robot.hasJoint(ctl.jointNames[size_t(i)])) { continue; }

  //   const size_t mbcIndex = robot.jointIndexByName(ctl.jointNames[size_t(i)]);

  //   if(q_mbc[mbcIndex].empty()) { continue; }

  //   // Position error
  //   double posError = ctl.q_rl(i) - q_mbc[mbcIndex][0];
  //   // // Velocity error
  //   double velError = -q_dot_mbc[mbcIndex][0];
  //   // Velocity error with real robot velocity
  //   ctl.velError_real(i) = q_dot_mbc[mbcIndex][0] - q_dot_real_mbc[mbcIndex][0];

  //   // // --- Passivity Observer / Passivity Controller on the q_c<->q coupling port ---
  //   // // Base coupling torque this port currently applies (kd term acting on the
  //   // // control/real velocity mismatch). This is the term whose energy we track.
  //   ctl.tauCoupling(i) = - ctl.kdCouple(i) * ctl.velError_real(i);

  //   auto & ps = ctl.passivityState[size_t(i)];
  //   // Power flowing through the port this step
  //   double P = ctl.tauCoupling(i) * ctl.velError_real(i);
  //   // Leaky energy accumulation
  //   ps.E = ps.leak * ps.E + P * dt;

  //   ctl.extraDamping(i) = 0.0;
  //   if(ps.E < 0.0 && std::abs(ctl.velError_real(i)) > relVelEps)
  //   {
  //     ctl.extraDamping(i) = -ps.E / (dt * ctl.velError_real(i) * ctl.velError_real(i));
  //     ctl.extraDamping(i) = std::min(ctl.extraDamping(i), ctl.passivityDampingMax[size_t(i)]); // safety clamp
  //     ps.E = 0.0; // energy debt repaid
  //   }
  //   ctl.tauCoupling(i) += -ctl.extraDamping(i) * ctl.velError_real(i);

  //   // PD control + passivity-corrected coupling term
  //   torqueTarget[ctl.jointNames[size_t(i)]] = {ctl.kp(i) * posError + ctl.kd(i) * velError + ctl.tauCoupling(i)};

  //   // torqueTarget[ctl.jointNames[size_t(i)]] = {ctl.kp(i) * posError + ctl.kd(i) * velError + ctl.kdCouple(i) *
  //   velError_real};
  // }

  // ctl.torqueTask->target(torqueTarget);
  return false;
}

void HRP5pRLQPController_RLPos::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  // ctl.solver().removeTask(ctl.torqueTask);
  ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.activateExternalTorqueComputation(false);
  ctl.activateFloatingBaseReal(false);
  ctl.activateContactConstraints(false);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_RLPos", HRP5pRLQPController_RLPos)
