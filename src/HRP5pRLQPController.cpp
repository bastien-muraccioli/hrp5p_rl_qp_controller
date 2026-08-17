#include "HRP5pRLQPController.h"

#include <RBDyn/MultiBodyConfig.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <fcntl.h>
#include <mc_joystick_plugin/joystick_inputs.h>
#include <termios.h>

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
  // Use joystick plugin if present else jeyboard inputs
  if(datastore().has("Joystick::connected") && datastore().get<bool>("Joystick::connected"))
    RLuseJoyStickInputs();
  else
    RLuseKeyboardInputs();

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

void HRP5pRLQPController::RLuseJoyStickInputs()
{
  // Get joystick functions
  auto & stickFunc = datastore().get<std::function<Eigen::Vector2d(joystickAnalogicInputs)>>("Joystick::Stick");

  // Read sticks values
  leftStick = stickFunc(joystickAnalogicInputs::L_STICK);
  // Apply dead zone
  double vel_x = 0.0;
  if(std::abs(leftStick(0) - 0.5) > joystickDeadZone)
  {
    vel_x = (leftStick(0) - 0.5) * 2.0 * maxVelCmd;
  }
  double vel_y = 0.0;
  if(std::abs(leftStick(1) - 0.5) > joystickDeadZone)
  {
    vel_y = (leftStick(1) - 0.5) * 2.0 * maxVelCmd;
  }

  rightStick = stickFunc(joystickAnalogicInputs::R_STICK);
  double yaw_cmd = 0.0;
  if(std::abs(rightStick(1) - 0.5) > joystickDeadZone)
  {
    yaw_cmd = (rightStick(1) - 0.5) * 2.0 * maxYawCmd;
  }

  // Read D-pad buttons
  DirectionButtons = {datastore().get<bool>("Joystick::UpPad"), datastore().get<bool>("Joystick::DownPad"),
                      datastore().get<bool>("Joystick::LeftPad"), datastore().get<bool>("Joystick::RightPad")};

  for(size_t i = 0; i < DirectionButtons.size(); ++i)
  {
    if(DirectionButtons[i])
    {
      switch(i)
      {
        case 0: // Up
          vel_x += 1.0 * maxVelCmd;
          break;
        case 1: // Down
          vel_x -= 1.0 * maxVelCmd;
          break;
        case 2: // Left
          vel_y += 1.0 * maxVelCmd;
          break;
        case 3: // Right
          vel_y -= 1.0 * maxVelCmd;
          break;
        default:
          break;
      }
    }
  }
  rlRuntime_.setCommand({vel_x, vel_y, yaw_cmd});
}

void HRP5pRLQPController::RLuseKeyboardInputs()
{
  struct Ctx
  {
    bool ready = false;
    termios old{};
    bool seen[4] = {};
    std::chrono::steady_clock::time_point ts[4];
    std::array<char, 64> buf{};
    size_t sz = 0;
  };
  static Ctx k;

  if(!k.ready)
  {
    if(::isatty(STDIN_FILENO) != 1) return;
    k.ready = true;
    ::tcgetattr(STDIN_FILENO, &k.old);
    termios raw = k.old;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = raw.c_cc[VTIME] = 0;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    ::fcntl(STDIN_FILENO, F_SETFL, ::fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
  }

  char tmp[32];
  ssize_t n = ::read(STDIN_FILENO, tmp, sizeof(tmp));
  if(n > 0 && k.sz + n < 64) std::copy(tmp, tmp + n, k.buf.begin() + k.sz), k.sz += n;

  auto now = std::chrono::steady_clock::now();
  for(size_t i = 0; i + 2 < k.sz; ++i)
    if(k.buf[i] == 27 && k.buf[i + 1] == '[')
    {
      int idx = k.buf[i + 2] == 'A'   ? 0
                : k.buf[i + 2] == 'B' ? 1
                : k.buf[i + 2] == 'D' ? 2
                : k.buf[i + 2] == 'C' ? 3
                                      : -1;
      if(idx >= 0) k.seen[idx] = true, k.ts[idx] = now;
      i += 2;
    }
  std::copy(k.buf.begin() + (k.sz > 2 ? k.sz - 2 : 0), k.buf.begin() + k.sz, k.buf.begin());
  k.sz = k.sz > 2 ? 2 : 0;

  const auto active = [&](int i)
  { return k.seen[i] && std::chrono::duration_cast<std::chrono::milliseconds>(now - k.ts[i]).count() < 500; };
  rlRuntime_.setCommand({(active(0) ? maxVelCmd : 0.0) - (active(1) ? maxVelCmd : 0.0),
                         (active(2) ? maxVelCmd : 0.0) - (active(3) ? maxVelCmd : 0.0), rlRuntime_.command()(2)});
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

    if(const auto & t = default_posture[robot().jointIndexByName(joint_name)]; !t.empty())
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
  logger().addLogEntry("HRP5pRLQPController_RL_phase", [this]() { return rlRuntime_.phase(); });
  logger().addLogEntry("HRP5pRLQPController_RL_policy_period_s", [this]() { return rlRuntime_.policyStepSize(); });
  logger().addLogEntry("HRP5pRLQPController_RL_update_count", [this]() { return rlRuntime_.policyUpdateCount(); });

  // Controller state variables
  logger().addLogEntry("HRP5pRLQPController_useQP", [this]() { return rlRuntime_.useQP(); });
  logger().addLogEntry("HRP5pRLQPController_torqueControl", [this]() { return rlRuntime_.isTorqueControl(); });
  logger().addLogEntry("HRP5pRLQPController_contactConstraints",
                       [this]() { return rlRuntime_.contactConstraintsAreEnabled(); });

  // Log current policy (name and convention)
  logger().addLogEntry("HRP5pRLQPController_currentPolicy", [this]() { return rlRuntime_.currentPolicyName(); });
  logger().addLogEntry("HRP5pRLQPController_observationConvention", [this]() { return rlRuntime_.conventionName(); });

  rlRuntime().addLogObs(*this);
}

void HRP5pRLQPController::addGui()
{
  gui()->addElement(
      {"HRP5pRLQPController", "Policy"},
      mc_rtc::gui::Label("Current policy", [this]() { return rlRuntime_.currentPolicyName(); }),
      mc_rtc::gui::Label("Current policy folder", [this]() { return rlRuntime_.currentPolicyFolder(); }),
      mc_rtc::gui::Label("Observation convention", [this]() { return rlRuntime_.conventionName(); }),
      mc_rtc::gui::Label("Observation source", [this]() { return rlRuntime_.observationSource(); }),
      mc_rtc::gui::Label("Base body", [this]() { return rlRuntime_.baseBody(); }),
      mc_rtc::gui::Label("Observation size", [this]() { return rlRuntime_.observationSize(); }),
      mc_rtc::gui::Label("Action size", [this]() { return rlRuntime_.actionSize(); }),
      mc_rtc::gui::Label("Controlled action size", [this]() { return rlRuntime_.controlledActionSize(); }),
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

  gui()->addElement({"HRP5pRLQPController", "Runtime"},
                    mc_rtc::gui::Label("Controller period [s]", [this]() { return timeStep; }),
                    mc_rtc::gui::Label("Controller rate [Hz]", [this]() { return 1.0 / timeStep; }),
                    mc_rtc::gui::Label("Policy period [s]", [this]() { return rlRuntime_.policyStepSize(); }),
                    mc_rtc::gui::Label("Policy rate [Hz]", [this]() { return rlRuntime_.policyRate(); }),
                    mc_rtc::gui::Label("Policy updates", [this]() { return rlRuntime_.policyUpdateCount(); }),
                    mc_rtc::gui::Label("Phase", [this]() { return rlRuntime_.phase(); }));

  gui()->addElement(
      {"HRP5pRLQPController", "Command"}, mc_rtc::gui::Label("Command values", []() { return std::string(" "); }),
      mc_rtc::gui::NumberInput(
          "vx", [this]() { return rlRuntime_.command()(0); }, [this](double v) { rlRuntime_.command()(0) = v; }),
      mc_rtc::gui::NumberInput(
          "vy", [this]() { return rlRuntime_.command()(1); }, [this](double v) { rlRuntime_.command()(1) = v; }),
      mc_rtc::gui::NumberInput(
          "yaw_rate", [this]() { return rlRuntime_.command()(2); }, [this](double v) { rlRuntime_.command()(2) = v; }),
      mc_rtc::gui::Label("Max values", []() { return std::string(" "); }),
      mc_rtc::gui::NumberInput(
          "max_vel_cmd", [this]() { return maxVelCmd; }, [this](double v) { maxVelCmd = v; }),
      mc_rtc::gui::NumberInput(
          "max_yaw_cmd", [this]() { return maxYawCmd; }, [this](double v) { maxYawCmd = v; }));
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
