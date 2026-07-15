#include "HRP5pRLQPController_Initial.h"

#include "../../include/HRP5pRLQPController.h"

void HRP5pRLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void HRP5pRLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController &>(ctl_);
  ctl.activateQPControl(true);
  ctl.rlRuntime().activateTorqueControl(false);
  ctl.rlRuntime().activateContactConstraints(true);
  if(!ctl.is_initial_posture_rl)
    ctl.postureTask->target(ctl.defaultPostureTarget);
  else
  {
    std::map<std::string, std::vector<double>> rl_q0;
    for(size_t i = 0; i < ctl.jointNames.size(); i++)
    {
      const std::string & joint = ctl.jointNames[i];
      if(ctl.defaultPostureTarget[joint].size() != 0) rl_q0[joint] = {ctl.rlRuntime().q_zero()[i]};
    }
    ctl.postureTask->target(rl_q0);
  }
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
