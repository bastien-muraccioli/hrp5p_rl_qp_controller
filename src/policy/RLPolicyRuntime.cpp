#include "policy/RLPolicyRuntime.h"

#include "NewRLQPController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace rlqp
{

RLPolicyRuntime::RLPolicyRuntime()
{
}

void RLPolicyRuntime::configure(const mc_rtc::Configuration & controllerConfig,
                                NewRLQPController & ctl,
                                const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  controllerConfig_ = controllerConfig;
  robotName_ = ctl.robot().name();
  controllerJointOrder_ = ctl.robot().refJointOrder();

  if(controllerConfig_.has("robot"))
  {
    baseBody_ = controllerConfig_("robot")("base_body", std::string("base_link"));
    observationSource_ = controllerConfig_("robot")("observation_source", std::string("realRobot"));
  }

  if(observationSource_ != "realRobot" && observationSource_ != "robot")
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] Invalid robot.observation_source '{}'. Expected 'realRobot' or 'robot'.",
      observationSource_);
  }

  const int nbActuatedJoints = static_cast<int>(controllerJointOrder_.size());

  q_rl_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  q_zero_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  currentActionScaled_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  actionScale_ = Eigen::VectorXd::Zero(nbActuatedJoints);

  kp_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kd_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kpBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kdBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);

  observationRegistry_ = makeDefaultObservationRegistry();

  policyManager_.load(controllerConfig_, ctl.jointNames);
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

void RLPolicyRuntime::reset(NewRLQPController & ctl)
{
  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  policyTimer_ = policyStepSize_;
}

void RLPolicyRuntime::runPolicyStepIfNeeded(NewRLQPController & ctl, double dt)
{
  if(!policyLoaded())
  {
    mc_rtc::log::error("[RLPolicyRuntime] Cannot run policy: no ONNX policy loaded");
    return;
  }

  policyTimer_ += dt;
  phaseElapsedTime_ += dt;

  if(phasePeriod_ > 0.0)
  {
    phaseNormalized_ = std::fmod(phaseElapsedTime_ / phasePeriod_, 1.0);
  }

  if(policyTimer_ < policyStepSize_) { return; }

  currentObservation_ = computeObservation(ctl);
  for (int i = 0; i < currentObservation_.size();i++)
    // mc_rtc::log::error("{} : {}", i, currentObservation_(i));

  if(currentObservation_.size() != policy_->getObservationSize())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] Observation size mismatch. ObservationManager produced {}, ONNX expects {}.",
      currentObservation_.size(),
      policy_->getObservationSize());
  }

  currentAction_ = policy_->predict(currentObservation_);

  if(currentAction_.size() != static_cast<int>(actionToControllerMap_.size()))
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] Action size mismatch. ONNX produced {}, active action mapping expects {} joints.",
      currentAction_.size(),
      actionToControllerMap_.size());
  }

  currentActionScaled_.setZero();
  q_rl_ = q_zero_;

  for(int actionIndex = 0; actionIndex < currentAction_.size(); ++actionIndex)
  {
    const int dofIndex = actionToControllerMap_[static_cast<size_t>(actionIndex)];

    currentActionScaled_(dofIndex) = actionScale_(dofIndex) * currentAction_(actionIndex);
    q_rl_(dofIndex) = currentActionScaled_(dofIndex) + q_zero_(dofIndex);
  }

  policyTimer_ = 0.0;
}

void RLPolicyRuntime::reloadCurrentPolicy(NewRLQPController & ctl,
                                          const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

void RLPolicyRuntime::loadNextPolicy(NewRLQPController & ctl,
                                     const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  policyManager_.selectNext();
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

int RLPolicyRuntime::observationSize() const
{
  if(policyLoaded()) { return policy_->getObservationSize(); }
  return static_cast<int>(currentObservation_.size());
}

int RLPolicyRuntime::actionSize() const
{
  if(policyLoaded()) { return policy_->getActionSize(); }
  return static_cast<int>(currentAction_.size());
}

void RLPolicyRuntime::setPDGainsRatio(double ratio,
                                      const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  if(ratio < 0.0)
  {
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] PD gains ratio must be non-negative, got {}", ratio);
  }

  pdGainsRatio_ = ratio;
  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = std::sqrt(pdGainsRatio_) * kdBase_;

  if(torqueTask)
  {
    torqueTask->setStiffness(kp_);
    torqueTask->setDamping(kd_);
  }
}

void RLPolicyRuntime::loadPolicy(const std::string & policyName,
                                 NewRLQPController & ctl,
                                 const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  policyManager_.select(policyName);

  const PolicyConfig & policy = policyManager_.current();

  mc_rtc::log::info("[RLPolicyRuntime] Loading policy '{}'", policy.name);

  configureControl(policy, ctl, torqueTask);

  const std::string conventionName =
    policy.observationsConfiguration("training_convention", std::string("mjlab"));
  activeConvention_ = ObservationConvention::fromConfig(controllerConfig_, conventionName);

  configureAction(policy, ctl);
  configureNetwork(policy);
  configureObservations(policy, ctl);

  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  validateObservationAgainstNetwork();

  policyTimer_ = policyStepSize_;

  mc_rtc::log::success(
    "[RLPolicyRuntime] Policy '{}' loaded. Observation size: {}, action size: {}",
    policy.name,
    currentObservation_.size(),
    currentAction_.size());
}

void RLPolicyRuntime::configureControl(const PolicyConfig & policy,
                                       NewRLQPController & ctl,
                                       const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  useQP_ = policy.useQP;
  policyStepSize_ = policy.policyStepSize;
  pdGainsRatio_ = policy.kpScale;

  phasePeriod_ = policy.observationsConfiguration("phase_period", 1.0);
  
  if(policy.policyConfiguration.has("control"))
  {
    const mc_rtc::Configuration control = policy.policyConfiguration("control");

    if(control.has("phase_period"))
    {
      control("phase_period", phasePeriod_);
    }
  }
  if(phasePeriod_ <= 0.0)
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] phase_period must be positive, got {}",
      policy.name,
      phasePeriod_);
  }

  kpBase_.setZero();
  kdBase_.setZero();

  kpBase_ = Eigen::Map<const Eigen::VectorXd>(policy.kp.data(), policy.kp.size());
  kdBase_ = Eigen::Map<const Eigen::VectorXd>(policy.kd.data(), policy.kp.size());

  kp_ = policy.kpScale * kpBase_;
  kd_ = policy.kdScale * kdBase_;

  if(torqueTask)
  {
    torqueTask->setStiffness(kp_);
    torqueTask->setDamping(kd_);
  }
}

void RLPolicyRuntime::configureAction(const PolicyConfig & policy,
                                      NewRLQPController & ctl)
{
  // Resolve action joints to controller indices (in RL convention order)
  mc_rtc::Configuration selector;
  if(!policy.actionJointGroup.empty())
  {
    mc_rtc::log::error(policy.actionJointGroup);
    mc_rtc::log::error(activeConvention_.jointGroups);
    selector.add("joints", policy.actionJointGroup);
  }

  // Fallback: all controller joints in controller order
  std::vector<int> fullFallback(controllerJointOrder_.size());
  std::iota(fullFallback.begin(), fullFallback.end(), 0);

  actionToControllerMap_ = activeConvention_.resolveJointControllerIndices(
    selector, controllerJointOrder_, fullFallback);

  q_zero_.setZero();
  q_rl_.setZero();
  actionScale_.setOnes();
  currentActionScaled_.setZero();

  const mc_rtc::Configuration action = policy.policyConfiguration("action");

  double scalarActionScale = 1.0;
  if(action.has("scale"))
  {
    try { action("scale", scalarActionScale); } catch(...) {}
  }

  //TODO stupid
  double scalarDefaultPosition = 0.0;
  if(action.has("default_position"))
  {
    try { action("default_position", scalarDefaultPosition); } catch(...) {}
  }

  size_t i = 0;
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = ctl.getPostureTask(ctl.robot().name());
  auto posture = FSMPostureTask->posture();
  for (const auto& j : ctl.robot().mb().joints()) {
    const std::string& joint_name = j.name();
    if (j.type() == rbd::Joint::Type::Rev) {
      if (const auto& t = posture[ctl.robot().jointIndexByName(joint_name)]; !t.empty()) {
        q_zero_(i) =t[0];
        i++;
      }
    }
  }

  for(size_t actionIndex = 0; actionIndex < actionToControllerMap_.size(); ++actionIndex)
  {
    const int dofIndex = actionToControllerMap_[actionIndex];
    const std::string & joint = controllerJointOrder_[static_cast<size_t>(dofIndex)];

    if(!ctl.robot().hasJoint(joint))
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] Resolved action joint '{}' does not exist on robot",
        policy.name,
        joint);
    }

    actionScale_(dofIndex) = scalarActionScale;
    //TODO make action scale work
    // std::map<std::string, double>::const_iterator scaleIt = policy.actionScale.find(joint);
    // if(scaleIt != policy.actionScale.end())
    // {
    //   actionScale_(dofIndex) = scaleIt->second;
    // }
    
    // std::map<std::string, double>::const_iterator defaultIt = policy.defaultPosition.find(joint);
    // if(defaultIt != policy.defaultPosition.end())
    // {
    //   q_zero_(dofIndex) = defaultIt->second;
    // }
  }
  mc_rtc::log::error(q_zero_);

  q_rl_ = q_zero_;
}

void RLPolicyRuntime::configureNetwork(const PolicyConfig & policy)
{
  try
  {
    policy_.reset(new RLPolicyInterface(policy.onnxPath));

    if(!policy_ || !policy_->isLoaded())
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] RL policy creation failed for '{}'",
        policy.name,
        policy.onnxPath);
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] Failed to load ONNX policy '{}': {}",
      policy.name,
      policy.onnxPath,
      e.what());
  }

  currentAction_ = Eigen::VectorXd::Zero(policy_->getActionSize());

  if(static_cast<int>(actionToControllerMap_.size()) != policy_->getActionSize())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] Resolved action joint count ({}) does not match ONNX action size ({})",
      policy.name,
      actionToControllerMap_.size(),
      policy_->getActionSize());
  }
}

void RLPolicyRuntime::configureObservations(const PolicyConfig & policy,
                                            NewRLQPController & ctl)
{
  observationManager_.load(policy.observationsConfiguration, controllerConfig_, observationRegistry_);
  observationManager_.configure(makeObservationContext(ctl));

  currentObservation_ = Eigen::VectorXd::Zero(policy_->getObservationSize());
}

void RLPolicyRuntime::resetObservationHistory(NewRLQPController & ctl)
{
  if(!policyLoaded())
  {
    mc_rtc::log::error("[RLPolicyRuntime] Cannot reset observation history: no policy loaded");
    return;
  }

  ObservationContext context = makeObservationContext(ctl);

  observationManager_.updateHistory(context);
  currentObservation_ = observationManager_.compute(context);
}

Eigen::VectorXd RLPolicyRuntime::computeObservation(NewRLQPController & ctl)
{
  ObservationContext context = makeObservationContext(ctl);
  return observationManager_.compute(context);
}

void RLPolicyRuntime::validateObservationAgainstNetwork() const
{
  if(!policyLoaded())
  {
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Cannot validate observation: no policy loaded");
  }

  if(currentObservation_.size() != policy_->getObservationSize())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] ObservationManager dimension ({}) does not match ONNX input size ({})",
      currentObservation_.size(),
      policy_->getObservationSize());
  }
}

ObservationContext RLPolicyRuntime::makeObservationContext(NewRLQPController & ctl)
{
  return ObservationContext{
    selectedObservationRobot(ctl),
    baseBody_,
    controllerJointOrder_,
    actionToControllerMap_,
    q_zero_,
    currentAction_,
    command_,
    phaseNormalized_,
    activeConvention_};
}

mc_rbdyn::Robot & RLPolicyRuntime::selectedObservationRobot(NewRLQPController & ctl)
{
  if(observationSource_ == "robot")
  {
    return ctl.robot();
  }

  return ctl.realRobot(robotName_);
}

double RLPolicyRuntime::mapValueOrThrow(const std::map<std::string, double> & values,
                                        const std::string & key,
                                        const std::string & mapName,
                                        const std::string & policyName) const
{
  std::map<std::string, double>::const_iterator it = values.find(key);

  if(it == values.end())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] Missing '{}' value for joint '{}'",
      policyName,
      mapName,
      key);
  }

  return it->second;
}

} // namespace rlqp
