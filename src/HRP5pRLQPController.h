#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueJointTask.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include "api.h"

#include "RLPolicyInterface.h"
#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <string>
#include <vector>


struct HRP5pRLQPController_DLLAPI HRP5pRLQPController : public mc_control::fsm::Controller
{
  HRP5pRLQPController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;
  void activateQPControl(bool activate);
  void activateTorqueControl(bool activate);
  void activateContactConstraints(bool activate);
  void activateExternalTorqueComputation(bool activate);
  void initializeRLObservation();

  // Task
  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;
  std::shared_ptr<mc_tasks::PostureTask> postureTask;
  std::map<std::string, std::vector<double>> defaultPostureTarget; // q0
  
  int nbActuatedJoints = 0;
  std::vector<std::string> jointNames;

  // Public RL related variables
  Eigen::VectorXd q_rl;
  Eigen::VectorXd q_zero;                      // Reference joint positions

  Eigen::VectorXd currentObservation;
  Eigen::VectorXd currentAction; // Raw output from the policy
  Eigen::VectorXd currentActionScaled; // Scaled action after applying actionScale, share the same size as q_rl, for the joints that are not controlled by the policy the value is 0 in currentActionScaled.

  Eigen::VectorXd actionScale;
  double policyStepSize;
  std::vector<std::string> refJointOrderRLAction;
  std::vector<int> actionToDofMap; // size = actionSize
  // std::vector<int> rlFrameworkToMcRtcJointMap; // size = nbActuatedJoints
  std::vector<int> mcRtcToRLFrameworkJointMap; // size = nbActuatedJoints

  size_t currentPolicyIndex = 0;
  std::unique_ptr<RLPolicyInterface> rlPolicy;
  utils utilsClass; // Utility functions for RL controller

  // observation
  static constexpr int HISTORY_SIZE = 5; // Number of past time steps to include in the observation
  std::array<Eigen::Vector3d, HISTORY_SIZE> linVel, angVel, projectedGravity;
  std::array<Eigen::VectorXd, HISTORY_SIZE> jointPos, jointVel, jointAction;
  std::array<Eigen::Vector3d, HISTORY_SIZE> velCmd; // Command vector [vx, vy, yaw_rate]
  std::array<Eigen::Vector6d, HISTORY_SIZE> footContactForces;

  Eigen::Vector3d currentVelCmd; // Current velocity command

  void setHighPDGains(bool high); 

private:
  mc_rtc::Configuration config_;
  // Add log entries readable by mc_log_ui 
  void addLog();
  // Add GUI elements to the mc_rtc GUI through Rviz and mc_mujoco
  void addGui();

  void initializeRobot();
  void configRL();
  void initializeRLPolicy();

  // Handle switching between Torque and Position control modes. Torque control is better for directly applying the RL torques, while position control is simulating the torque reference in high gains position control which is experimental. Except in simulation avoid switching between control modes during the execution on the real robot to prevent potential issues with the hardware.
  bool manageModeSwitching();
  // Directly use RL output without QP modifications (Torque Control only) 
  bool byPassQPControl(); 
  void computeLimits();
  bool printLimits_ = true;

  Eigen::VectorXd externalTorques_;

  std::string robotName_;

  // Mode switching
  bool useQP_ = true;
  bool isTorqueControl_ = false;
  bool isFloatingBaseReal_ = false;
  bool controlModeChanged_ = false;

  // Constraint configuration
  double velPercent_ = 0.99; // Percentage of the max velocity taking account in the joint velocity constraint.
  double dsPercent_ = 0.01; // Percentage of the max joint range taking account in the joint position limit constraint.
  double diPercent_ = 0.1; // Doesn't matter since di > ds. This variable is not used in the constraint dynamics.

  // CBF Gains More details are explained in the paper cf. Readme.md. 
  // Must be tuned depending on the robot.
  double zeta_jointLimit_ = 1.2;
  double lambda_jointLimit_ = 200.0; // Same gain for joint position limits and velocity limits. 
  double zeta_selfCollision_ = 1.2;
  double lambda_selfCollision_ = 100.0; 

  // Gains
  double pdGainsRatio_ = 1.0;
  Eigen::VectorXd kp_;  // Gains set to the robot/simulator = pd_gains_ratio * kp_base
  Eigen::VectorXd kd_;  // Gains set to the robot/simulator = pd_gains_ratio * kd_base
  Eigen::VectorXd kpBase_; // Base RL PD gains from config
  Eigen::VectorXd kdBase_; // Base RL PD gains from config
  Eigen::VectorXd highKpBase_; // Base High gain PD gains from config
  Eigen::VectorXd highKdBase_; // Base High gain PD gains from config

  // RL
  std::vector<std::string> policyPaths_;
  Eigen::VectorXd tau_rl_;
  Eigen::VectorXd qdot_rl_integrated_;
  Eigen::VectorXd q_rl_integrated_;

  sva::PTransformd control_qfb_;
  sva::PTransformd real_qfb_;
  sva::MotionVecd control_vfb_;
  sva::MotionVecd real_vfb_;

  std::map<std::string, double> q0_map_; // Used to create the mc_rtc to RL framework joint mapping

  bool contactModeChanged_ = true;
  bool contactConstraintsAreEnabled_ = true;
};
