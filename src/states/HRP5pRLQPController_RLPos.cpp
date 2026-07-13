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
  if(ctl.postureTask->inSolver()) ctl.solver().removeTask(ctl.postureTask);
  ctl.setHighPDGains(false); // Use RL gains
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
}

bool HRP5pRLQPController_RLPos::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  // ctl.updateFootContactsFromForceSensors();
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  return false;
}

void HRP5pRLQPController_RLPos::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.activateExternalTorqueComputation(false);
  ctl.activateFloatingBaseReal(false);
  ctl.activateContactConstraints(false);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_RLPos", HRP5pRLQPController_RLPos)
