#include "HRP5pRLQPController_Initial.h"

#include "../../include/HRP5pRLQPController.h"

void HRP5pRLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.rlRuntime().activateTorqueControl(false);
  ctl.rlRuntime().activateContactConstraints(true);
  ctl.postureTask->target(ctl.defaultPostureTarget);
  mc_rtc::log::warning("default {}", ctl.defaultPostureTarget);
  mc_rtc::log::error("rl {}", ctl.rlRuntime().q_zero());
  // ctl.postureTask->target(ctl.rlRuntime().q_zero());
  for (const auto &joint_name : ctl.robot().refJointOrder())
  ctl.postureTask->target(ctl.rlRuntime().q_zero());
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
