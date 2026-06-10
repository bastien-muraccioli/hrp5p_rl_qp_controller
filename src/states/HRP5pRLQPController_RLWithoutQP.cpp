#include "HRP5pRLQPController_RLWithoutQP.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_RLWithoutQP::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_RLWithoutQP::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(false);
  ctl.activateTorqueControl(true);
  ctl.activateContactConstraints(false);
  ctl.solver().addTask(ctl.torqueJointTask);
  ctl.setHighPDGains(false); // Use RL gains
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
}

bool HRP5pRLQPController_RLWithoutQP::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  return false;
}

void HRP5pRLQPController_RLWithoutQP::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_RLWithoutQP", HRP5pRLQPController_RLWithoutQP)
