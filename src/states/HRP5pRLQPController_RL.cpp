#include "HRP5pRLQPController_RL.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_RL::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_RL::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.activateTorqueControl(true);
  ctl.activateContactConstraints(false);
  ctl.activateExternalTorqueComputation(true);
  ctl.solver().addTask(ctl.torqueJointTask);
  if(ctl.postureTask->inSolver()) ctl.solver().removeTask(ctl.postureTask);
  ctl.setHighPDGains(false); // Use RL gains
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
}

bool HRP5pRLQPController_RL::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  return false;
}

void HRP5pRLQPController_RL::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.activateExternalTorqueComputation(false);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_RL", HRP5pRLQPController_RL)
