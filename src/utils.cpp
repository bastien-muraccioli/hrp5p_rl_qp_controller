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
          // mc_rtc::log::info("currentAction[{}] = {}, scaled action[{}] for joint {}: {}", j, ctl.currentAction(j), i, ctl.jointNames[i], ctl.currentActionScaled(i));
      }
      // Run new inference and update target position, scaled by action scale
      ctl.q_rl = ctl.q_zero + ctl.currentActionScaled;
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

  // Observation
  auto & real_robot = ctl.realRobot(ctl.robots()[0].name());
  auto & robot = ctl.robots()[0];
  auto & imu = ctl.robot().bodySensor("Accelerometer");

  auto q_map = real_robot.encoderValues();
  auto q_dot_map = real_robot.encoderVelocities();

  Eigen::VectorXd q = Eigen::VectorXd::Map(q_map.data(), int(q_map.size()));
  Eigen::VectorXd q_dot = Eigen::VectorXd::Map(q_dot_map.data(), int(q_dot_map.size()));

  Eigen::VectorXd q_rl_framework_ordered = Eigen::VectorXd::Zero(ctl.dofNumber);
  Eigen::VectorXd q_dot_rl_framework_ordered = Eigen::VectorXd::Zero(ctl.dofNumber);
  Eigen::VectorXd q_0_rl_framework_ordered = Eigen::VectorXd::Zero(ctl.dofNumber);
  for (size_t i = 0; i < ctl.mcRtcToRLFrameworkJointMap.size(); ++i) {
    int rl_index = ctl.mcRtcToRLFrameworkJointMap[i];
    if (rl_index >= 0 && rl_index < ctl.dofNumber) {
      q_rl_framework_ordered(rl_index) = q(i);
      q_dot_rl_framework_ordered(rl_index) = q_dot(i);
      q_0_rl_framework_ordered(rl_index) = ctl.q_zero(i);
    } else {
      mc_rtc::log::warning("Joint '{}' at index {} in mc_rtc order is not mapped to RL framework joint index. This joint will be ignored in the observation.", ctl.jointNames[i], i);
    }
  }

  int offset = 0;
  auto append = [&](const Eigen::VectorXd& v) {
    obs.segment(offset, v.size()) = v;
    offset += v.size();
  };

  switch (ctl.currentPolicyIndex) {
    case 0:
    {
      // shift history: t-2 <- t-1 <- t
      for (int i = ctl.HISTORY_SIZE - 1; i > 0; --i) {
          ctl.angVel[i] = ctl.angVel[i - 1];
          ctl.projectedGravity[i] = ctl.projectedGravity[i - 1];
          ctl.velCmd[i] = ctl.velCmd[i - 1];
          ctl.jointPos[i] = ctl.jointPos[i - 1];
          ctl.jointVel[i] = ctl.jointVel[i - 1];
          ctl.jointAction[i] = ctl.jointAction[i - 1];
      }

      // insert newest value at t=0
      // ctl.angVel[0] = imu.angularVelocity();
      Eigen::VectorXd floatingBase_alphaIn = rbd::paramToVector(robot.mb(), robot.mbc().alpha);
      ctl.angVel[0] = floatingBase_alphaIn.segment(0, 3);
      Eigen::VectorXd floatingBase_qIn = rbd::paramToVector(real_robot.mb(), real_robot.mbc().q);
      // Suppose you have the IMU orientation (rotation from IMU to world)
      Eigen::VectorXd q_imu_vector = floatingBase_qIn.segment(0, 4);
      Eigen::Quaterniond q_imu_to_world = Eigen::Quaterniond::Identity();
      q_imu_to_world.w() = q_imu_vector(0);
      q_imu_to_world.x() = q_imu_vector(1);
      q_imu_to_world.y() = q_imu_vector(2);
      q_imu_to_world.z() = q_imu_vector(3);

      Eigen::Matrix3d R_world_to_imu = q_imu_to_world.toRotationMatrix();
      Eigen::Vector3d gravity_b = R_world_to_imu.transpose() * Eigen::Vector3d(0.0, 0.0, -9.81);

      ctl.projectedGravity[0] = gravity_b.normalized();
      ctl.velCmd[0] = ctl.currentVelCmd;
      ctl.jointPos[0] = q_rl_framework_ordered - q_0_rl_framework_ordered;
      ctl.jointVel[0] = q_dot_rl_framework_ordered;
      ctl.jointAction[0] = ctl.currentAction;

      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.angVel[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.projectedGravity[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.jointPos[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.jointVel[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.jointAction[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.velCmd[i]);
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

      // insert newest value at t=0
      Eigen::VectorXd floatingBase_alphaIn = rbd::paramToVector(robot.mb(), robot.mbc().alpha);
      ctl.linVel[0] = floatingBase_alphaIn.segment(3, 3);
      ctl.angVel[0] = floatingBase_alphaIn.segment(0, 3);

      Eigen::VectorXd floatingBase_qIn = rbd::paramToVector(real_robot.mb(), real_robot.mbc().q);
      // Suppose you have the IMU orientation (rotation from IMU to world)
      Eigen::VectorXd q_imu_vector = floatingBase_qIn.segment(0, 4);
      Eigen::Quaterniond q_imu_to_world = Eigen::Quaterniond::Identity();
      q_imu_to_world.w() = q_imu_vector(0);
      q_imu_to_world.x() = q_imu_vector(1);
      q_imu_to_world.y() = q_imu_vector(2);
      q_imu_to_world.z() = q_imu_vector(3);

      Eigen::Matrix3d R_world_to_imu = q_imu_to_world.toRotationMatrix();
      Eigen::Vector3d gravity_b = R_world_to_imu.transpose() * Eigen::Vector3d(0.0, 0.0, -9.81);

      ctl.projectedGravity[0] = gravity_b.normalized();
      ctl.velCmd[0] = ctl.currentVelCmd;
      ctl.jointPos[0] = q_rl_framework_ordered - q_0_rl_framework_ordered;
      ctl.jointVel[0] = q_dot_rl_framework_ordered;
      ctl.jointAction[0] = ctl.currentAction;

      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.linVel[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.angVel[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.projectedGravity[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.jointPos[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.jointVel[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.jointAction[i]);
      for (int i = 0; i < ctl.HISTORY_SIZE; ++i) append(ctl.velCmd[i]);
      break;
    }
    default:
    {
      mc_rtc::log::error("Unknown policy index: {}", ctl.currentPolicyIndex);
      break;
    }
  }
  
  return obs;
}
