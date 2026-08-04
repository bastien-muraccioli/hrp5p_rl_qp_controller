#include "HRP5pRLQPController_Initial.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.activateTorqueControl(false);
  ctl.activateContactConstraints(true);
  ctl.postureTask->target(ctl.defaultPostureTarget);
  ctl.solver().addTask(ctl.postureTask);
}

bool HRP5pRLQPController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  // ctl.updateFootContactsFromForceSensors();
  return false;
}

void HRP5pRLQPController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.solver().removeTask(ctl.postureTask);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_Initial", HRP5pRLQPController_Initial)
