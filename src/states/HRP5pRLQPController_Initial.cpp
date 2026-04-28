#include "HRP5pRLQPController_Initial.h"

#include "../HRP5pRLQPController.h"

void HRP5pRLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
}

bool HRP5pRLQPController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  output("OK");
  return true;
}

void HRP5pRLQPController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
}

EXPORT_SINGLE_STATE("HRP5pRLQPController_Initial", HRP5pRLQPController_Initial)
