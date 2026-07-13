#include "HRP5pRLQPController_RL.h"

#include "../../include/HRP5pRLQPController.h"

void HRP5pRLQPController_RL::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_RL::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.rlRuntime().activateTorqueControl(true);
  ctl.rlRuntime().activateContactConstraints(false);
  ctl.activateExternalTorqueComputation(true);
  ctl.solver().addTask(ctl.torqueJointTask);
  if(ctl.postureTask->inSolver()) ctl.solver().removeTask(ctl.postureTask);
  ctl.torqueTask_setHighPDGains(false); // Use RL gains
  ctl.rlStateRunner.start(ctl, "RL_State");
}

bool HRP5pRLQPController_RL::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.rlStateRunner.run(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.rlRuntime().q_rl());
  return false;
}

void HRP5pRLQPController_RL::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.activateExternalTorqueComputation(false);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_RL", HRP5pRLQPController_RL)
