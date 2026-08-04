#include "HRP5pRLQPController_TorquePos.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_TorquePos::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_TorquePos::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.activateTorqueControl(false);
  ctl.activateContactConstraints(true);
  ctl.activateExternalTorqueComputation(false);
  ctl.solver().addTask(ctl.torqueJointTask);
  if(ctl.postureTask->inSolver()) ctl.solver().removeTask(ctl.postureTask);
  ctl.torqueJointTask->setPosTarget(ctl.q_zero);
  ctl.setHighPDGains(true); // Use high PD gains
}

bool HRP5pRLQPController_TorquePos::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  return false;
}

void HRP5pRLQPController_TorquePos::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.activateContactConstraints(false);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_TorquePos", HRP5pRLQPController_TorquePos)
