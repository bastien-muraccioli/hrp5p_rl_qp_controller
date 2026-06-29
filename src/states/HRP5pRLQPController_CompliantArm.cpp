#include "HRP5pRLQPController_CompliantArm.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_CompliantArm::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_CompliantArm::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.activateTorqueControl(false);
  ctl.activateContactConstraints(true);
  ctl.postureTask->reset();
  ctl.compliantPostureTask->reset();
  ctl.postureTask->selectUnactiveJoints(ctl.solver(), activeJoints_);
  ctl.compliantPostureTask->selectActiveJoints(ctl.solver(), activeJoints_);
  ctl.solver().addTask(ctl.compliantPostureTask);
  ctl.solver().addTask(ctl.postureTask);
}

bool HRP5pRLQPController_CompliantArm::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  return false;
}

void HRP5pRLQPController_CompliantArm::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.postureTask->reset();
  ctl.compliantPostureTask->reset();
  ctl.solver().removeTask(ctl.compliantPostureTask);
  ctl.solver().removeTask(ctl.postureTask);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_CompliantArm", HRP5pRLQPController_CompliantArm)
