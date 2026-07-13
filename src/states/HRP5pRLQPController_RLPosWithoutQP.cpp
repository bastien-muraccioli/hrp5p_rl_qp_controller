#include "HRP5pRLQPController_RLPosWithoutQP.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_RLPosWithoutQP::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_RLPosWithoutQP::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(false);
  ctl.activateTorqueControl(false);
  ctl.activateContactConstraints(false);
  ctl.activateExternalTorqueComputation(true);
  // ctl.solver().addTask(ctl.torqueJointTask);
  ctl.setHighPDGains(false); // Use RL gains
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
}

bool HRP5pRLQPController_RLPosWithoutQP::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  // ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  return false;
}

void HRP5pRLQPController_RLPosWithoutQP::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  // ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.activateExternalTorqueComputation(false);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_RLPosWithoutQP", HRP5pRLQPController_RLPosWithoutQP)
