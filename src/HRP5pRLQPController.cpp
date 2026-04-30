#include "HRP5pRLQPController.h"
#include <mc_rtc/gui/ArrayInput.h>

#include <RBDyn/MultiBodyConfig.h>

HRP5pRLQPController::HRP5pRLQPController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;
  currentPolicyIndex = size_t(config_("default_policy_index", 0));
  
  //Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {zeta_selfCollision_, lambda_selfCollision_});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);
  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));
  // Initialize Task
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(
      solver(), robot().robotIndex(), 100.0, 1);
  solver().addTask(torqueJointTask);

  initializeRobot();
  initializeRLPolicy();

  addGui();
  addLog();
  mc_rtc::log::success("HRP5pRLQPController init done");
}

bool HRP5pRLQPController::run()
{
  bool run = manageModeSwitching();
  if(byPassQPControl()) // Run RL without taking the QP into account
  {
    return true;
  }
  return run; // Return false if QP fails
}

void HRP5pRLQPController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void HRP5pRLQPController::initializeRobot()
{
  useQP_ = config_("policies")[currentPolicyIndex]("use_QP", true);
  isTorqueControl_ = config_("policies")[currentPolicyIndex]("is_torque_control", false);
  if(isTorqueControl_)
  {
    mc_rtc::log::info("[HRP5pRLQPController] Using Torque Control mode");
    datastore().make<std::string>("ControlMode", "Torque");
  }
  else
  {
    mc_rtc::log::info("[HRP5pRLQPController] Using Position Control mode");
    datastore().make<std::string>("ControlMode", "Position");
  }

  // get the joints order (urdf) depending on the robot used
  robotName_ = robot().name();
  // Remove the floating base joints if they exist
  dofNumber = (robot().mb().nrJoints() > 0 && robot().mb().joint(0).type() == rbd::Joint::Free)
          ? robot().mb().nrDof() - 6
          : robot().mb().nrDof();

  q_rl = Eigen::VectorXd::Zero(dofNumber);
  q_zero = Eigen::VectorXd::Zero(dofNumber);
  actionScale = Eigen::VectorXd::Zero(dofNumber);
  currentActionScaled = Eigen::VectorXd::Zero(dofNumber);
  kp_ = Eigen::VectorXd::Zero(dofNumber);
  kd_ = Eigen::VectorXd::Zero(dofNumber);
  kpBase_ = Eigen::VectorXd::Zero(dofNumber);
  kdBase_ = Eigen::VectorXd::Zero(dofNumber);
  
  // Get the gains from the configuration or set default values
  pdGainsRatio_ = config_("policies")[currentPolicyIndex]("pd_gains_ratio", 1.0);
  std::map<std::string, double> actionScale_map = config_("policies")[currentPolicyIndex]("action_scale");
  std::map<std::string, double> kp_map = config_("policies")[currentPolicyIndex]("kp");
  std::map<std::string, double> kd_map = config_("policies")[currentPolicyIndex]("kd");
  // std::map<std::string, double> q0_map = config_("policies")[currentPolicyIndex]("q0");

  auto q0_cfg = config_("policies")[currentPolicyIndex]("q0");
  auto keys = q0_cfg.keys(); // this preserves order
  
  auto currentPos = robot().mbc().q; // Current joint positions from the robot state
  int i = 0;
  for (const auto & key : keys) 
  { 
    std::string joint_name = key;
    jointNames.emplace_back(joint_name);
    kpBase_[i] = kp_map.at(joint_name);
    kdBase_[i] = kd_map.at(joint_name);
    q_zero[i] = q0_cfg(key);
    q_rl[i] = currentPos[robot().jointIndexByName(joint_name)][0];
    actionScale[i] = actionScale_map.at(joint_name);
    i++;
  }

  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = pdGainsRatio_ * kdBase_;
  torqueJointTask->setStiffness(kp_);
  torqueJointTask->setDamping(kd_);
}

void HRP5pRLQPController::initializeRLPolicy()
{
  // load policy specific configuration
  policyPaths_ = config_("policy_path", std::vector<std::string>{"walking_better_h1.onnx"});
  configRL();

  currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
  int actionSize = rlPolicy->getActionSize();
  currentAction = Eigen::VectorXd::Zero(actionSize);

  Eigen::VectorXd floatingBase_qIn = rbd::paramToVector(robot().mb(), robot().mbc().q);
  // Suppose you have the IMU orientation (rotation from IMU to world)
  Eigen::VectorXd q_imu_vector = floatingBase_qIn.segment(0, 4);
  Eigen::Quaterniond q_imu_to_world = Eigen::Quaterniond::Identity();
  q_imu_to_world.w() = q_imu_vector(0);
  q_imu_to_world.x() = q_imu_vector(1);
  q_imu_to_world.y() = q_imu_vector(2);
  q_imu_to_world.z() = q_imu_vector(3);

  Eigen::Matrix3d R_world_to_imu = q_imu_to_world.toRotationMatrix();
  Eigen::Vector3d gravity_b = R_world_to_imu.transpose() * Eigen::Vector3d(0.0, 0.0, -9.81);

  // Observation initialization
  Eigen::Vector3d initProjectedGravity = gravity_b.normalized();   

  currentVelCmd.setZero();

  Eigen::VectorXd q_rl_framework_ordered = Eigen::VectorXd::Zero(dofNumber);
  Eigen::VectorXd q_0_rl_framework_ordered = Eigen::VectorXd::Zero(dofNumber);
  for (size_t i = 0; i < mcRtcToRLFrameworkJointMap.size(); ++i) {
    int rl_index = mcRtcToRLFrameworkJointMap[i];
    if (rl_index >= 0 && rl_index < dofNumber) {
      q_rl_framework_ordered(rl_index) = q_rl(i);
      q_0_rl_framework_ordered(rl_index) = q_zero(i);
    } else {
      mc_rtc::log::warning("Joint '{}' at index {} in mc_rtc order is not mapped to RL framework joint index. This joint will be ignored in the observation.", jointNames[i], i);
    }
  }

  // Initialize all history slots
  for (int i = 0; i < HISTORY_SIZE; ++i) {
      linVel[i].setZero();
      angVel[i].setZero();
      projectedGravity[i] = initProjectedGravity;
      velCmd[i].setZero();
      jointPos[i] = q_rl_framework_ordered - q_0_rl_framework_ordered; // Start with current joint positions
      jointVel[i] = Eigen::VectorXd::Zero(dofNumber);
      jointAction[i] = Eigen::VectorXd::Zero(actionSize);
  }
}

void HRP5pRLQPController::switchPolicy(int policyIndex)
{
  if(policyIndex < 0 || policyIndex >= static_cast<int>(policyPaths_.size())) {
    mc_rtc::log::error("[HRP5pRLQPController] Invalid policy index: {}", policyIndex);
    return;
  }
  
  mc_rtc::log::info("[HRP5pRLQPController] Switching from policy [{}] to policy [{}]", currentPolicyIndex, policyIndex);
  currentPolicyIndex = size_t(policyIndex);
  
  // Update policy-specific boolean flags
  useQP_ = config_("policies")[currentPolicyIndex]("use_QP", true);
  isTorqueControl_ = config_("policies")[currentPolicyIndex]("is_torque_control", false);
  if(isTorqueControl_) datastore().get<std::string>("ControlMode") = "Torque";
  else datastore().get<std::string>("ControlMode") = "Position";

  configRL();

  // Update PD gains
  pdGainsRatio_ = config_("policies")[currentPolicyIndex]("pd_gains_ratio", 1.0);
  std::map<std::string, double> kp_map = config_("policies")[currentPolicyIndex]("kp");
  std::map<std::string, double> kd_map = config_("policies")[currentPolicyIndex]("kd");
  std::map<std::string, double> q0_map = config_("policies")[currentPolicyIndex]("q0");
  std::map<std::string, double> actionScale_map = config_("policies")[currentPolicyIndex]("action_scale");

  for(int i = 0; i < dofNumber; ++i) {
    const auto & jName = robot().mb().joint(static_cast<int>(i + 1)).name();  // +1 to skip Root
    if(kp_map.count(jName)) {
      kpBase_(i) = kp_map[jName];
    }
    if(kd_map.count(jName)) {
      kdBase_(i) = kd_map[jName];
    }
    if(q0_map.count(jName)) {
      q_zero[i] = q0_map[jName];
    }
    if(actionScale_map.count(jName)) {
      actionScale[i] = actionScale_map[jName];
    }
  }
  // Update PD gains
  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = pdGainsRatio_ * kdBase_;
  torqueJointTask->setStiffness(kp_);
  torqueJointTask->setDamping(kd_);
}

bool HRP5pRLQPController::byPassQPControl()
{
  if(useQP_) return false; // QP is not bypassed, do nothing
  if(!isTorqueControl_)
  {
    mc_rtc::log::warning("[HRP5pRLQPController] QP can't be bypassed in position control mode. Please enable torque control to bypass QP.");
    return false;
  }

  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  auto tau = robot().mbc().jointTorque;
  auto q_map = robot().encoderValues();
  auto q_dot_map = robot().encoderVelocities();

  Eigen::VectorXd q = Eigen::VectorXd::Map(q_map.data(), int(q_map.size()));
  Eigen::VectorXd q_dot = Eigen::VectorXd::Map(q_dot_map.data(), int(q_dot_map.size()));
  Eigen::VectorXd tau_rl = (kp_).cwiseProduct(q_rl - q) - (kd_).cwiseProduct(q_dot);
  
  int i = 0;
  for (const auto &joint_name : jointNames)
  {
    tau[robot().jointIndexByName(joint_name)][0] = tau_rl[i];
    i++;
  }
  // Update joint torques 
  robot().mbc().jointTorque = tau;
  return true;
}

void HRP5pRLQPController::addLog()
{
  // Robot State variables
  logger().addLogEntry("HRP5pRLQPController_kp_base", [this]() { return kpBase_; });
  logger().addLogEntry("HRP5pRLQPController_kd_base", [this]() { return kdBase_; });
  logger().addLogEntry("HRP5pRLQPController_kp_current", [this]() { return kp_; });
  logger().addLogEntry("HRP5pRLQPController_kd_current", [this]() { return kd_; });
  logger().addLogEntry("HRP5pRLQPController_pd_gains_ratio", [this]() { return pdGainsRatio_; });

  // RL variables
  logger().addLogEntry("HRP5pRLQPController_RL_q", [this]() { return q_rl; });
  logger().addLogEntry("HRP5pRLQPController_RL_qZero", [this]() { return q_zero; });
  logger().addLogEntry("HRP5pRLQPController_RL_currentObservation", [this]() { return currentObservation; });
  logger().addLogEntry("HRP5pRLQPController_RL_currentAction", [this]() { return currentAction; });
  logger().addLogEntry("HRP5pRLQPController_RL_currentActionScaled", [this]() { return currentActionScaled; });
  logger().addLogEntry("HRP5pRLQPController_RL_actionScale", [this]() { return actionScale; });
  
  // Controller state variables
  logger().addLogEntry("HRP5pRLQPController_useQP", [this]() { return useQP_; });
  logger().addLogEntry("HRP5pRLQPController_isTorqueControl", [this]() { return isTorqueControl_; });

  // Log current policy (combined index and path)
  logger().addLogEntry("HRP5pRLQPController_currentPolicy", [this]() { 
    return std::to_string(currentPolicyIndex) + ": " + policyPaths_[currentPolicyIndex]; 
  });

  // Log observation
  for (int i = 0; i < HISTORY_SIZE; ++i) {
    logger().addLogEntry("HRP5pRLQPController_obs_linVel_" + std::to_string(i), [this, i]() { return linVel[i]; });
    logger().addLogEntry("HRP5pRLQPController_obs_angVel_" + std::to_string(i), [this, i]() { return angVel[i]; });
    logger().addLogEntry("HRP5pRLQPController_obs_projectedGravity_" + std::to_string(i), [this, i]() { return projectedGravity[i]; });
    logger().addLogEntry("HRP5pRLQPController_obs_velCmd_" + std::to_string(i), [this, i]() { return velCmd[i]; });
    logger().addLogEntry("HRP5pRLQPController_obs_jointPos_" + std::to_string(i), [this, i]() { return jointPos[i]; });
    logger().addLogEntry("HRP5pRLQPController_obs_jointVel_" + std::to_string(i), [this, i]() { return jointVel[i]; });
    logger().addLogEntry("HRP5pRLQPController_obs_jointAction_" + std::to_string(i), [this, i]() { return jointAction[i]; });
  }
}

void HRP5pRLQPController::addGui()
{
  gui()->addElement({"HRP5pRLQPController", "Policy"},
  mc_rtc::gui::Label("Current policy", [this]() -> const std::string & 
    { 
      return policyPaths_[currentPolicyIndex]; 
    }),
    mc_rtc::gui::ComboInput(
      "Select policy",
      policyPaths_,
      [this]() -> const std::string & 
      { 
        return policyPaths_[currentPolicyIndex]; 
      },
      [this](const std::string & selected) 
      {  // Capture config by VALUE (makes a safe copy)
        // Find the index of the selected policy
        auto it = std::find(policyPaths_.begin(), policyPaths_.end(), selected);
        if(it != policyPaths_.end()) 
        {
          int newIndex = static_cast<int>(std::distance(policyPaths_.begin(), it));
          mc_rtc::log::info("[HRP5pRLQPController] User requested policy switch to [{}]: {}", newIndex, selected);
          // Switch to new policy without reinitializing robot
          switchPolicy(newIndex);
        }
      }),
    mc_rtc::gui::Button("Reload current policy", [this]() 
    {
      mc_rtc::log::info("[HRP5pRLQPController] User requested to reload current policy [{}]", currentPolicyIndex);
      switchPolicy(int(currentPolicyIndex));
    })
  );

  // Add PD gains ratio slider
  gui()->addElement({"HRP5pRLQPController", "PD Gains"},
    mc_rtc::gui::NumberSlider(
      "PD Gains Ratio", [this]() { return pdGainsRatio_; },
      [this](double v) { 
        pdGainsRatio_ = v;
        kp_ = pdGainsRatio_ * kpBase_;
        kd_ = pdGainsRatio_ * kdBase_;
        torqueJointTask->setStiffness(kp_);
        torqueJointTask->setDamping(kd_);
      }, 0.0, 2.0),
    mc_rtc::gui::Label("Current kp", kp_),
    mc_rtc::gui::Label("Current kd", kd_)
  );

  gui()->addElement({"ControlMode"}, 
    mc_rtc::gui::Button("Switch Control Mode", [this]()
      {
        controlModeChanged_ = true;
        isTorqueControl_ = !isTorqueControl_;
      }),
      mc_rtc::gui::Label("Current Control Mode", [this]()
        {
          return isTorqueControl_ ? "Torque Control" : "Position Control";
        }),
      mc_rtc::gui::Button("Toggle QP Control", [this]()
        {
          useQP_ = !useQP_;
        }),
      mc_rtc::gui::Label("QP Control", [this]()
      {
        return useQP_ ? "Enforced" : "Bypassed";
      })
    );
  
    gui()->addElement({"HRP5pRLQPController", "Velocity Command"},
      mc_rtc::gui::ArrayInput("Current Velocity Command", {"vx", "vy", "yaw_rate"}, currentVelCmd));
}

void HRP5pRLQPController::configRL()
{
  mc_rtc::log::info("[HRP5pRLQPController] Loading RL policy [{}]: {}", currentPolicyIndex, policyPaths_[currentPolicyIndex]);
  try {
    rlPolicy = std::make_unique<RLPolicyInterface>(policyPaths_[currentPolicyIndex]);
    if(rlPolicy) {
      mc_rtc::log::success("[HRP5pRLQPController] RL policy loaded successfully");
      // Initialize observation vector with the correct size from the loaded policy
      currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
      mc_rtc::log::info("[HRP5pRLQPController] Initialized observation vector with size: {}", rlPolicy->getObservationSize());
      currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());
      mc_rtc::log::info("[HRP5pRLQPController] Initialized action vector with size: {}", rlPolicy->getActionSize());
    } else {
      mc_rtc::log::error_and_throw("[HRP5pRLQPController] RL policy creation failed - policy is null");
    }
  } catch(const std::exception& e) {
    mc_rtc::log::error_and_throw("[HRP5pRLQPController] Failed to load RL policy: {}", e.what());
  }

  policyStepSize = config_("policies")[currentPolicyIndex]("policy_step_size", 1.0);
  int physicsStepSize = config_("policies")[currentPolicyIndex]("physics_step_size", 1.0);
  if(physicsStepSize - timeStep > 1e-6) {
    mc_rtc::log::warning("[HRP5pRLQPController] Physics step size ({:.3f} s) is larger than controller time step ({:.3f} s). This may cause issues with the policy. Consider fixing the controller time step.", physicsStepSize, timeStep);
  }

  refJointOrder = config_("policies")[currentPolicyIndex]("ref_joint_order", std::vector<std::string>{});
  if(refJointOrder.size() != size_t(rlPolicy->getActionSize())) {
    mc_rtc::log::error_and_throw("[HRP5pRLQPController] Reference joint order size ({}) does not match policy action size ({}). Please check the configuration.", refJointOrder.size(), rlPolicy->getActionSize());
  }

  // Create mapping from action indices to robot joint indices based on the reference joint order
  actionToDofMap.resize(refJointOrder.size(), -1); // Initialize with -1 to indicate unmapped actions
  for (size_t j = 0; j < refJointOrder.size(); ++j) {
    for (int i = 0; i < dofNumber; ++i) {
      if (jointNames[i] == refJointOrder[j]) {
        actionToDofMap[j] = i;
        // mc_rtc::log::info("[HRP5pRLQPController] Mapping action index {} to joint[{}] '{}'", j, i, jointNames[i]);
        break;
      }
    }
  }

  // Create mapping between RL framework joint order and mc_rtc joint indices, this is useful for correctly ordering the observation and action vectors according to the robot's joint order in mc_rtc.
  auto action_scale_cfg = config_("policies")[currentPolicyIndex]("action_scale");
  auto keys = action_scale_cfg.keys(); // this preserves order

  if (keys.size() != static_cast<size_t>(dofNumber)) {
    mc_rtc::log::error_and_throw("[HRP5pRLQPController] The number of joints in action_scale config ({}) does not match the robot's dof number ({}). Please check the configuration.", keys.size(), dofNumber);
  }

  // rlFrameworkToMcRtcJointMap.resize(dofNumber, -1); // Initialize with -1 to indicate unmapped joints
  mcRtcToRLFrameworkJointMap.resize(dofNumber, -1);
  int j = 0;
  for (const auto & key : keys) { 
    for (int i = 0; i < dofNumber; ++i) {
      if (jointNames[i] == key) {
        // rlFrameworkToMcRtcJointMap[j] = i;
        mcRtcToRLFrameworkJointMap[i] = j;
        break;
      }
    }
    j++;
  }

  for (int i = 0; i < dofNumber; ++i) {
    if (mcRtcToRLFrameworkJointMap[i] == -1) {
      mc_rtc::log::error_and_throw("[HRP5pRLQPController] Joint '{}' was not properly mapped!", jointNames[i]);
    }
  }
}

bool HRP5pRLQPController::manageModeSwitching()
{
  if(controlModeChanged_)
  {
    if(isTorqueControl_)
    {
      mc_rtc::log::info("[HRP5pRLQPController] Switching to Torque Control");
      datastore().assign<std::string>("ControlMode", "Torque");
    }
    else
    {
      mc_rtc::log::info("[HRP5pRLQPController] Switching to Position Control");
      datastore().assign<std::string>("ControlMode", "Position");
    }
    controlModeChanged_ = false;
  }

  if(isTorqueControl_)
  {
    return mc_control::fsm::Controller::run(
          mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
  else 
  {
    return mc_control::fsm::Controller::run();
  }
}