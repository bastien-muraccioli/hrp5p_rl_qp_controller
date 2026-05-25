#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <mc_rtc/logging.h>

#include "HRP5pRLQPController.h"

void utils::start_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
{
  auto & ctl = static_cast<HRP5pRLQPController&>(ctl_);
  state_name_ = state_name;
  mc_rtc::log::info("{} state started", state_name);

  syncTime_ = ctl.policyStepSize;
    
  if(!ctl.rlPolicy || !ctl.rlPolicy->isLoaded())
  {
    mc_rtc::log::error("RL policy not loaded in {} state", state_name);
    return;
  }

  ctl.gui()->addElement(
    {"HRP5pRLQPController", state_name},
    mc_rtc::gui::Label("Policy Loaded", [&ctl]() { 
      return ctl.rlPolicy->isLoaded() ? "Yes" : "No"; 
    }),
    mc_rtc::gui::Label("Observation Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy->getObservationSize()); 
    }),
    mc_rtc::gui::Label("Action Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy->getActionSize()); 
    })
  );

  ctl.initializeRLObservation();

  mc_rtc::log::success("{} state initialization completed", state_name);
}

void utils::run_rl_state(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController&>(ctl_);
  try
  {
    syncTime_ += ctl.timeStep;
    if(syncTime_ >= ctl.policyStepSize)
    {
      ctl.currentObservation = getCurrentObservation(ctl);
      ctl.currentAction = ctl.rlPolicy->predict(ctl.currentObservation);
      for (int j = 0; j < ctl.currentAction.size(); ++j) {
          int i = ctl.actionToDofMap[j];
          ctl.currentActionScaled(i) = ctl.actionScale(i) * ctl.currentAction(j);
          // mc_rtc::log::info("joint({}) {}: action scale {} * action {} -> scaled action {}", i, ctl.jointNames[i], ctl.actionScale(i), ctl.currentAction(j), ctl.currentActionScaled(i));
          ctl.q_rl(i) = ctl.q_zero(i) + ctl.currentActionScaled(i);
          mc_rtc::log::info("joint({}) {}: q_zero {} + currentActionScaled {} -> q_rl {}", i, ctl.jointNames[i], ctl.q_zero(i), ctl.currentActionScaled(i), ctl.q_rl(i));
      }
      // Run new inference and update target position, scaled by action scale
      // ctl.q_rl = ctl.q_zero + ctl.currentActionScaled;
      syncTime_ = 0.0;
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("Error during RL state run: {}", e.what());
  }
}

void utils::teardown_rl_state(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeCategory({"HRP5pRLQPController", state_name_});
}

Eigen::VectorXd utils::getCurrentObservation(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HRP5pRLQPController&>(ctl_);
  Eigen::VectorXd obs(ctl.rlPolicy->getObservationSize());
  obs = Eigen::VectorXd::Zero(ctl.rlPolicy->getObservationSize());

  int offset = 0;
  auto appendToObs = [&](const Eigen::VectorXd& v) {
    obs.segment(offset, v.size()) = v;
    offset += v.size();
  };

  switch (ctl.currentPolicyIndex) {
    case 0:
    {
      constexpr int WALK_HISTORY = 3; // history depth for walk policy (indices 0,1,2)

      // shift: move t-1 -> t-2, t -> t-1
      for (int i = WALK_HISTORY - 1; i > 0; --i) {
          ctl.linVel[i]          = ctl.linVel[i - 1];
          ctl.angVel[i]          = ctl.angVel[i - 1];
          ctl.projectedGravity[i]= ctl.projectedGravity[i - 1];
          ctl.velCmd[i]          = ctl.velCmd[i - 1];
          ctl.jointPos[i]        = ctl.jointPos[i - 1];
          ctl.jointVel[i]        = ctl.jointVel[i - 1];
          ctl.jointAction[i]     = ctl.jointAction[i - 1];
      }

      ctl.initializeRLObservation(); // update t with current observation

      for (int i = WALK_HISTORY - 1; i >= 0; --i) appendToObs(ctl.linVel[i]);
      for (int i = WALK_HISTORY - 1; i >= 0; --i) appendToObs(ctl.angVel[i]);
      for (int i = WALK_HISTORY - 1; i >= 0; --i) appendToObs(ctl.projectedGravity[i]);
      for (int i = WALK_HISTORY - 1; i >= 0; --i) appendToObs(ctl.jointPos[i]);
      for (int i = WALK_HISTORY - 1; i >= 0; --i) appendToObs(ctl.jointVel[i]);
      for (int i = WALK_HISTORY - 1; i >= 0; --i) appendToObs(ctl.jointAction[i]);
      for (int i = WALK_HISTORY - 1; i >= 0; --i) appendToObs(ctl.velCmd[i]);
      break;
    }
    case 1:
    {
      // shift history: t-2 <- t-1 <- t
      for (int i = ctl.HISTORY_SIZE - 1; i > 0; --i) {
          ctl.linVel[i] = ctl.linVel[i - 1];
          ctl.angVel[i] = ctl.angVel[i - 1];
          ctl.projectedGravity[i] = ctl.projectedGravity[i - 1];
          ctl.velCmd[i] = ctl.velCmd[i - 1];
          ctl.jointPos[i] = ctl.jointPos[i - 1];
          ctl.jointVel[i] = ctl.jointVel[i - 1];
          ctl.jointAction[i] = ctl.jointAction[i - 1];
      }

      ctl.initializeRLObservation(); // update t with current observation

      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.linVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.angVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.projectedGravity[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointPos[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointAction[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.velCmd[i]);
      break;
    }
    case 2:
    {
      // shift history: t-2 <- t-1 <- t
      for (int i = ctl.HISTORY_SIZE - 1; i > 0; --i) {
          ctl.linVel[i] = ctl.linVel[i - 1];
          ctl.angVel[i] = ctl.angVel[i - 1];
          ctl.projectedGravity[i] = ctl.projectedGravity[i - 1];
          ctl.velCmd[i] = ctl.velCmd[i - 1];
          ctl.jointPos[i] = ctl.jointPos[i - 1];
          ctl.jointVel[i] = ctl.jointVel[i - 1];
          ctl.jointAction[i] = ctl.jointAction[i - 1];
          ctl.footContactForces[i] = ctl.footContactForces[i - 1];
      }

      ctl.initializeRLObservation(); // update t with current observation

      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.linVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.angVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.projectedGravity[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointPos[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.footContactForces[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointAction[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.velCmd[i]);
      break;
    }
    default:
    {
      mc_rtc::log::error("Unknown policy index: {}", ctl.currentPolicyIndex);
      break;
    }
  }

  assert(offset == obs.size() && "Observation size mismatch: written bytes != allocated size");
  return obs;
}
