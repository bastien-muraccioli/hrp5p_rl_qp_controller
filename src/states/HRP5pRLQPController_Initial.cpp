#include "HRP5pRLQPController_Initial.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
}

bool HRP5pRLQPController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  return false;
}

void HRP5pRLQPController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.utilsClass.teardown_rl_state(ctl);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_Initial", HRP5pRLQPController_Initial)
