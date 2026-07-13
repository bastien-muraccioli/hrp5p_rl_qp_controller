#pragma once

#include "observation/Observation.h"
#include "observation/ObservationManager.h"
#include "policy/PolicyConfig.h"
#include "policy/RLPolicyInterface.h"

#include <Eigen/Core>

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/Configuration.h>
#include <mc_tasks/TorqueJointTask.h>

#include <memory>
#include <string>
#include <vector>

struct HRP5pRLQPController;

namespace rlqp
{

/**
 * @brief Runtime owner of the active RL policy session.
 *
 * Responsibilities:
 *
 * - Load and switch policies.
 * - Own the ONNX inference backend.
 * - Own the ObservationManager.
 * - Build observation vectors.
 * - Execute policy inference.
 * - Convert policy outputs into q_rl targets.
 * - Manage policy gains and action scaling.
 *
 * HRP5pRLQPController remains responsible for the mc_rtc lifecycle.
 * RLPolicyRuntime owns everything that is policy-dependent.
 */
class RLPolicyRuntime
{
public:
  RLPolicyRuntime();

  void configure(const mc_rtc::Configuration & controllerConfig,
                 HRP5pRLQPController & ctl,
                 const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void reset(HRP5pRLQPController & ctl);
  void runPolicyStepIfNeeded(HRP5pRLQPController & ctl, double dt);

  void reloadCurrentPolicy(HRP5pRLQPController & ctl, const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void loadPolicyByName(const std::string & policyName,
                        HRP5pRLQPController & ctl,
                        const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void loadNextPolicy(HRP5pRLQPController & ctl, const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  bool policyLoaded() const
  {
    return policy_ && policy_->isLoaded();
  }

  const std::string & currentPolicyName() const
  {
    return policyManager_.currentName();
  }
  const std::vector<std::string> & availablePolicyNames() const
  {
    return policyManager_.names();
  }
  const std::string & currentPolicyFolder() const
  {
    return policyManager_.current().folder;
  }
  const std::string & conventionName() const
  {
    return observationManager_.conventionName();
  }

  int observationSize() const;
  int actionSize() const;

  double policyStepSize() const
  {
    return policyStepSize_;
  }
  double phase() const
  {
    return phaseNormalized_;
  }

  bool useQP() const
  {
    return useQP_;
  }
  void setUseQP(bool useQP)
  {
    useQP_ = useQP;
  }

  bool isTorqueControl() const
  {
    return isTorqueControl_;
  }

  const Eigen::VectorXd & q_rl() const
  {
    return q_rl_;
  }
  const Eigen::VectorXd & q_zero() const
  {
    return q_zero_;
  }
  const Eigen::VectorXd & currentObservation() const
  {
    return currentObservation_;
  }
  const Eigen::VectorXd & currentAction() const
  {
    return currentAction_;
  }
  const Eigen::VectorXd & currentActionScaled() const
  {
    return currentActionScaled_;
  }
  const Eigen::VectorXd & actionScale() const
  {
    return actionScale_;
  }

  const Eigen::VectorXd & kp() const
  {
    return kp_;
  }
  const Eigen::VectorXd & kd() const
  {
    return kd_;
  }
  const Eigen::VectorXd & kpBase() const
  {
    return kpBase_;
  }
  const Eigen::VectorXd & kdBase() const
  {
    return kdBase_;
  }
  Eigen::VectorXd & highKpBase()
  {
    return highKpBase_;
  }
  Eigen::VectorXd & highKdBase()
  {
    return highKdBase_;
  }

  bool controlModeChanged() const
  {
    return controlModeChanged_;
  }
  void setControlModeChanged(bool new_control_mode)
  {
    controlModeChanged_ = new_control_mode;
  }
  bool isFloatingBaseReal() const
  {
    return isFloatingBaseReal_;
  }
  void setIsFloatingBaseReal(bool new_value)
  {
    isFloatingBaseReal_ = new_value;
  }

  bool contactModeChanged() const
  {
    return contactModeChanged_;
  }
  void setContactModeChanged(bool new_value)
  {
    contactModeChanged_ = new_value;
  }
  bool contactConstraintsAreEnabled() const
  {
    return contactConstraintsAreEnabled_;
  }
  void setContactConstraintsAreEnabled(bool new_value)
  {
    contactConstraintsAreEnabled_ = new_value;
  }

  double pdGainsRatio() const
  {
    return pdGainsRatio_;
  }

  void setPDGainsRatio(double ratio, const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  Eigen::Vector3d & command()
  {
    return command_;
  }
  const Eigen::Vector3d & command() const
  {
    return command_;
  }

  void setHighPDGains(bool high);

  void activateContactConstraints(bool activate);
  void activateTorqueControl(bool activate);

private:
  void loadPolicy(const std::string & policyName,
                  HRP5pRLQPController & ctl,
                  const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void configureControl(const PolicyConfig & policy,
                        HRP5pRLQPController & ctl,
                        const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void configureAction(const PolicyConfig & policy, HRP5pRLQPController & ctl);
  void configureNetwork(const PolicyConfig & policy);
  void configureObservations(const PolicyConfig & policy, HRP5pRLQPController & ctl);

  void resetObservationHistory(HRP5pRLQPController & ctl);
  Eigen::VectorXd computeObservation(HRP5pRLQPController & ctl);

  void validateObservationAgainstNetwork() const;
  ObservationContext makeObservationContext(HRP5pRLQPController & ctl);
  mc_rbdyn::Robot & selectedObservationRobot(HRP5pRLQPController & ctl);

private:
  mc_rtc::Configuration controllerConfig_;

  /** Available policy configurations. */
  PolicyManager policyManager_;
  /** Observation type factory registry. */
  ObservationRegistry observationRegistry_;
  /** Active observation pipeline. */
  ObservationManager observationManager_;
  /** Active training-environment convention. */
  ObservationConvention activeConvention_;
  /** Active ONNX policy instance. */
  std::unique_ptr<RLPolicyInterface> policy_;

  std::string robotName_;
  std::string baseBody_ = "base_link";
  std::string observationSource_ = "realRobot";

  std::vector<std::string> controllerJointOrder_;
  /** @brief Maps active policy action index to controllerJointOrder_ index. policyJointControllerIndices in
   * ObservationContext. */
  std::vector<int> actionToControllerMap_;

  /** @brief Controller-order joint indices that are actually allowed to receive the policy action. */
  std::vector<int> controlledActionControllerIndices_;

  Eigen::VectorXd q_rl_;
  Eigen::VectorXd q_zero_;
  Eigen::VectorXd currentObservation_;
  Eigen::VectorXd currentAction_;
  Eigen::VectorXd currentActionScaled_;
  Eigen::VectorXd actionScale_;

  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd kpBase_;
  Eigen::VectorXd kdBase_;
  Eigen::VectorXd highKpBase_; // Base High gain PD gains from config
  Eigen::VectorXd highKdBase_; // Base High gain PD gains from config

  bool controlModeChanged_ = false;
  bool isFloatingBaseReal_ = false;

  bool contactModeChanged_ = true;
  bool contactConstraintsAreEnabled_ = true;

  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();

  bool useQP_ = true;
  bool isTorqueControl_ = false;

  double policyStepSize_ = 0.02;
  double policyTimer_ = 0.0;

  double phasePeriod_ = 1.0;
  double phaseElapsedTime_ = 0.0;
  double phaseNormalized_ = 0.0;

  double pdGainsRatio_ = 1.0;
};

} // namespace rlqp
