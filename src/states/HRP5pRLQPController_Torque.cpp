#include "HRP5pRLQPController_Torque.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_Torque::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_Torque::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.activateTorqueControl(true);
  ctl.activateContactConstraints(false);
  ctl.activateExternalTorqueComputation(true);
  ctl.solver().addTask(ctl.torqueJointTask);
  if(ctl.postureTask->inSolver()) ctl.solver().removeTask(ctl.postureTask);
  ctl.torqueJointTask->setPosTarget(ctl.q_zero);
  ctl.setHighPDGains(true); // Use high PD gains
}

bool HRP5pRLQPController_Torque::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  return false;
}

void HRP5pRLQPController_Torque::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.activateExternalTorqueComputation(false);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_Torque", HRP5pRLQPController_Torque)
