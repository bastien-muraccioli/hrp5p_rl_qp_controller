#include "HRP5pRLQPController.h"

#include <RBDyn/MultiBodyConfig.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <cmath>

HRP5pRLQPController::HRP5pRLQPController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;

  // Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {zeta_selfCollision_, lambda_selfCollision_});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(
      robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Initialize Tasks
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(solver(), robot().robotIndex(), 100.0, 1000);
  postureTask = getPostureTask(robot().name());
  compliantPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 100.0, 1);

  initializeRobotBasics(config);

  rlRuntime_.configure(config_, *this, torqueJointTask);

  addGui();
  addLog();

  mc_rtc::log::success("[HRP5pRLQPController] init done");
}

bool HRP5pRLQPController::run()
{
  if(printLimits_) computeLimits();
  auto & robot = robots()[0];
  auto & real_robot = realRobot(robot.name());
  if(rlRuntime_.contactModeChanged())
  {
    if(rlRuntime_.contactConstraintsAreEnabled())
    {
      Eigen::Vector6d footcontact_dof = Eigen::Vector6d::Ones();
      // footcontact_dof = Eigen::Vector6d(0, 0, 1, 0, 0, 0);
      // Eigen::Vector6d footcontact_dof = Eigen::Vector6d::Zero();
      // footcontact_dof.head<3>() = Eigen::Vector3d::Ones(); // Only enforce the position constraint for the foot
      // contact
      addContact({robot.name(), "ground", "RightFootCenter", "AllGround", 0.7, footcontact_dof});
      addContact({robot.name(), "ground", "LeftFootCenter", "AllGround", 0.7, footcontact_dof});
    }
    else
    {
      clearContacts();
    }
    rlRuntime_.setContactModeChanged(false);
  }

  // robot.tvmRobot().setRealState(real_robot.mbc().q, real_robot.mbc().alpha);
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
  rlRuntime_.reset(*this);
}

void HRP5pRLQPController::activateQPControl(bool activate)
{
  rlRuntime_.setUseQP(activate);
}

rlqp::RLPolicyRuntime & HRP5pRLQPController::rlRuntime()
{
  return rlRuntime_;
}

const rlqp::RLPolicyRuntime & HRP5pRLQPController::rlRuntime() const
{
  return rlRuntime_;
}

void HRP5pRLQPController::initializeRobotBasics(const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("[HRP5pRLQPController] Using torque control mode");

  if(!datastore().has("ControlMode"))
  {
    datastore().make<std::string>("ControlMode", "Position");
  }
  else
  {
    datastore().assign<std::string>("ControlMode", "Position");
  }

  robotName_ = robot().name();
  jointNames = robot().refJointOrder();
  nbActuatedJoints = static_cast<int>(jointNames.size());

  std::map<std::string, double> highKp_map = config("high_kp");
  std::map<std::string, double> highKd_map = config("high_kd");
  is_initial_posture_rl = config("is_initial_posture_rl", false);

  std::vector<std::vector<double>> default_posture = postureTask->posture();
  auto & highKpBase = rlRuntime_.highKpBase();
  highKpBase.resize(jointNames.size());
  auto & highKdBase = rlRuntime_.highKdBase();
  highKdBase.resize(jointNames.size());

  int i = 0;
  for(const auto & joint_name : jointNames)
  {
    highKpBase[static_cast<Eigen::Index>(i)] = highKp_map.at(joint_name);
    highKdBase[static_cast<Eigen::Index>(i)] = highKd_map.at(joint_name);

    if (const auto& t = default_posture[robot().jointIndexByName(joint_name)]; !t.empty())
      defaultPostureTarget[joint_name] = t;

    i++;
  }
}

bool HRP5pRLQPController::byPassQPControl()
{
  if(rlRuntime_.useQP()) return false; // QP is not bypassed, do nothing

  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  Eigen::VectorXd tau_rl = Eigen::VectorXd::Zero(nbActuatedJoints);
  const std::vector<std::vector<double>> & q_mbc = robot().mbc().q;
  const std::vector<std::vector<double>> & q_dot_mbc = robot().mbc().alpha;

  int i = 0;
  for(const auto & joint_name : jointNames)
  {
    const double q = q_mbc[robot().jointIndexByName(joint_name)][0];
    const double q_dot = q_dot_mbc[robot().jointIndexByName(joint_name)][0];
    tau_rl(i) = rlRuntime_.kp()(i) * (rlRuntime_.q_rl()(i) - q) - rlRuntime_.kd()(i) * q_dot;
    robot().mbc().jointTorque[robot().jointIndexByName(joint_name)][0] = tau_rl(i);
    i++;
  }

  return true;
}

void HRP5pRLQPController::addLog()
{
  // Robot State variables
  logger().addLogEntry("HRP5pRLQPController_kp_base", [this]() { return rlRuntime_.kpBase(); });
  logger().addLogEntry("HRP5pRLQPController_kd_base", [this]() { return rlRuntime_.kdBase(); });
  logger().addLogEntry("HRP5pRLQPController_kp_current", [this]() { return rlRuntime_.kp(); });
  logger().addLogEntry("HRP5pRLQPController_kd_current", [this]() { return rlRuntime_.kd(); });
  logger().addLogEntry("HRP5pRLQPController_pd_gains_ratio", [this]() { return rlRuntime_.pdGainsRatio(); });

  // RL variables
  logger().addLogEntry("HRP5pRLQPController_RL_q", [this]() { return rlRuntime_.q_rl(); });
  logger().addLogEntry("HRP5pRLQPController_RL_qZero", [this]() { return rlRuntime_.q_zero(); });
  logger().addLogEntry("HRP5pRLQPController_RL_currentObservation",
                       [this]() { return rlRuntime_.currentObservation(); });
  logger().addLogEntry("HRP5pRLQPController_RL_currentAction", [this]() { return rlRuntime_.currentAction(); });
  logger().addLogEntry("HRP5pRLQPController_RL_currentActionScaled",
                       [this]() { return rlRuntime_.currentActionScaled(); });
  logger().addLogEntry("HRP5pRLQPController_RL_actionScale", [this]() { return rlRuntime_.actionScale(); });
  logger().addLogEntry("HRP5pRLQPController_RL_command", [this]() { return rlRuntime_.command(); });

  // Controller state variables
  logger().addLogEntry("HRP5pRLQPController_useQP", [this]() { return rlRuntime_.useQP(); });

  // Log current policy (name and convention)
  logger().addLogEntry("HRP5pRLQPController_currentPolicy", [this]() { return rlRuntime_.currentPolicyName(); });
  logger().addLogEntry("HRP5pRLQPController_observationConvention", [this]() { return rlRuntime_.conventionName(); });
}

void HRP5pRLQPController::addGui()
{
  gui()->addElement(
      {"HRP5pRLQPController", "Policy"},
      mc_rtc::gui::Label("Current policy", [this]() { return rlRuntime_.currentPolicyName(); }),
      mc_rtc::gui::Label("Current policy folder", [this]() { return rlRuntime_.currentPolicyFolder(); }),
      mc_rtc::gui::Label("Observation convention", [this]() { return rlRuntime_.conventionName(); }),
      mc_rtc::gui::ComboInput(
          "Select policy", rlRuntime_.availablePolicyNames(), [this]() { return rlRuntime_.currentPolicyName(); },
          [this](const std::string & policyName) { rlRuntime_.loadPolicyByName(policyName, *this, torqueJointTask); }),
      mc_rtc::gui::Button("Reload current policy",
                          [this]() { rlRuntime_.reloadCurrentPolicy(*this, torqueJointTask); }));

  gui()->addElement({"HRP5pRLQPController", "PD Gains"},
                    mc_rtc::gui::NumberSlider(
                        "PD Gains Ratio", [this]() { return rlRuntime_.pdGainsRatio(); },
                        [this](double v) { rlRuntime_.setPDGainsRatio(v, torqueJointTask); }, 0.0, 2.0),
                    mc_rtc::gui::Label("Current kp", [this]() { return rlRuntime_.kp(); }),
                    mc_rtc::gui::Label("Current kd", [this]() { return rlRuntime_.kd(); }));

  gui()->addElement(
      {"HRP5pRLQPController", "Control"},
      mc_rtc::gui::Button("Toggle QP Control", [this]() { rlRuntime_.setUseQP(!rlRuntime_.useQP()); }),
      mc_rtc::gui::Label("QP Control", [this]() { return rlRuntime_.useQP() ? "Enforced" : "Bypassed"; }),
      mc_rtc::gui::Button("Toggle print joint limits", [this]() { printLimits_ = !printLimits_; }),
      mc_rtc::gui::Label("Print joint limits", [this]() { return printLimits_ ? "Enabled" : "Disabled"; }));

  gui()->addElement(
      {"HRP5pRLQPController", "Command"},
      mc_rtc::gui::NumberInput(
          "vx", [this]() { return rlRuntime_.command()(0); }, [this](double v) { rlRuntime_.command()(0) = v; }),
      mc_rtc::gui::NumberInput(
          "vy", [this]() { return rlRuntime_.command()(1); }, [this](double v) { rlRuntime_.command()(1) = v; }),
      mc_rtc::gui::NumberInput(
          "yaw_rate", [this]() { return rlRuntime_.command()(2); }, [this](double v) { rlRuntime_.command()(2) = v; }));
}

void HRP5pRLQPController::computeLimits()
{
  const double epsilon = 1e-5;

  auto & robot = robots()[0];
  const auto & currentPos = robot.q();
  const auto & currentVel = robot.alpha();
  const auto & currentTau = robot.jointTorque();

  const auto & qLimLower = robot.ql();
  const auto & qLimUpper = robot.qu();

  const auto & qDotLimLower = robot.vl();
  const auto & qDotLimUpper = robot.vu();

  const auto & tauLimLower = robot.tl();
  const auto & tauLimUpper = robot.tu();

  for(std::string joint : robot.refJointOrder())
  {
    // Skip joints not present in the multibody chain (e.g. finger joints)
    if(!robot.hasJoint(joint))
    {
      continue;
    }

    int i = robot.jointIndexByName(joint);

    // Skip fixed joints or multi-dof joints (no scalar limits)
    if(qLimLower[i].empty() || qLimUpper[i].empty())
    {
      continue;
    }

    const double ds = dsPercent_ * (qLimUpper[i][0] - qLimLower[i][0]);
    const double posLimitUp = qLimUpper[i][0] - ds;
    const double posLimitLow = qLimLower[i][0] + ds;
    const double velLimitUp = velPercent_ * qDotLimUpper[i][0];
    const double velLimitLow = velPercent_ * qDotLimLower[i][0];
    const double tauLimitUp = tauLimUpper[i][0];
    const double tauLimitLow = tauLimLower[i][0];

    if(currentPos[i][0] > posLimitUp + epsilon)
    {
      mc_rtc::log::warning("Joint {} position upper limit breached: currentPos = {}, limit = {}", joint,
                           currentPos[i][0], posLimitUp);
    }
    if(currentPos[i][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning("Joint {} position lower limit breached: currentPos = {}, limit = {}", joint,
                           currentPos[i][0], posLimitLow);
    }
    if(currentVel[i][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning("Joint {} velocity upper limit breached: currentVel = {}, limit = {}", joint,
                           currentVel[i][0], velLimitUp);
    }
    if(currentVel[i][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning("Joint {} velocity lower limit breached: currentVel = {}, limit = {}", joint,
                           currentVel[i][0], velLimitLow);
    }
    if(currentTau[i][0] > tauLimitUp + epsilon)
    {
      mc_rtc::log::warning("Joint {} torque upper limit breached: currentTau = {}, limit = {}", joint, currentTau[i][0],
                           tauLimitUp);
    }
    if(currentTau[i][0] < tauLimitLow - epsilon)
    {
      mc_rtc::log::warning("Joint {} torque lower limit breached: currentTau = {}, limit = {}", joint, currentTau[i][0],
                           tauLimitLow);
    }
  }
}

bool HRP5pRLQPController::manageModeSwitching()
{
  if(rlRuntime_.controlModeChanged())
  {
    if(rlRuntime_.isTorqueControl())
    {
      mc_rtc::log::info("[HRP5pRLQPController] Switching to Torque Control");
      datastore().assign<std::string>("ControlMode", "Torque");
    }
    else
    {
      mc_rtc::log::info("[HRP5pRLQPController] Switching to Position Control");
      datastore().assign<std::string>("ControlMode", "Position");
    }

    rlRuntime_.setControlModeChanged(false);
  }

  if(rlRuntime_.isTorqueControl())
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }

  if(rlRuntime_.isFloatingBaseReal())
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoopWithRealFloatingBase);
  }
  else
  {
    return mc_control::fsm::Controller::run();
  }
}

void HRP5pRLQPController::torqueTask_setHighPDGains(bool high)
{
  rlRuntime_.setHighPDGains(high);
  torqueJointTask->setStiffness(rlRuntime_.kp());
  torqueJointTask->setDamping(rlRuntime_.kd());
}

void HRP5pRLQPController::activateExternalTorqueComputation(bool activate)
{
  if(activate != datastore().call<bool>("EF_Estimator::isActive"))
  {
    datastore().call("EF_Estimator::toggleActive");
  }
}

void HRP5pRLQPController::updateFootContactsFromForceSensors()
{
  auto & real_robot = realRobot(robots()[0].name());

  // ── 1. Measure force norms ──────────────────────────────────────────────
  leftFootForceNorm_ = real_robot.forceSensor("LeftFootForceSensor").worldWrench(real_robot).force().norm();
  rightFootForceNorm_ = real_robot.forceSensor("RightFootForceSensor").worldWrench(real_robot).force().norm();

  // ── 2. Independent Schmitt triggers ────────────────────────────────────
  const bool leftActive = leftFootSchmitt_.update(leftFootForceNorm_);
  const bool rightActive = rightFootSchmitt_.update(rightFootForceNorm_);

  // ── 3. Determine desired contact state ─────────────────────────────────
  bool wantLeft = false;
  bool wantRight = false;

  if(leftActive && rightActive)
  {
    const double diff = leftFootForceNorm_ - rightFootForceNorm_;
    if(std::abs(diff) <= footForceEpsilon_)
    {
      // Similar load → double support
      wantLeft = true;
      wantRight = true;
    }
    else if(diff > footForceEpsilon_)
    {
      // Left significantly heavier → left support only
      wantLeft = true;
      wantRight = false;
    }
    else
    {
      // Right significantly heavier → right support only
      wantLeft = false;
      wantRight = true;
    }
  }
  else if(leftActive)
  {
    wantLeft = true;
    wantRight = false;
  }
  else if(rightActive)
  {
    wantLeft = false;
    wantRight = true;
  }
  // else: neither active → both false (already default)

  // ── 4. Apply changes only when state flips ──────────────────────────────
  const Eigen::Vector6d footcontact_dof = Eigen::Vector6d::Ones();

  if(wantLeft != prevLeftContact_)
  {
    if(wantLeft)
      addContact({robot().name(), "ground", "LeftFootCenter", "AllGround", 0.7, footcontact_dof});
    else
      removeContact({robot().name(), "ground", "LeftFootCenter", "AllGround"});
    prevLeftContact_ = wantLeft;
  }

  if(wantRight != prevRightContact_)
  {
    if(wantRight)
      addContact({robot().name(), "ground", "RightFootCenter", "AllGround", 0.7, footcontact_dof});
    else
      removeContact({robot().name(), "ground", "RightFootCenter", "AllGround"});
    prevRightContact_ = wantRight;
  }
}
