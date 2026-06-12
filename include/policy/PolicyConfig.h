#pragma once

#include <mc_rtc/Configuration.h>

#include <map>
#include <string>
#include <vector>

namespace rlqp
{

/**
 * @brief Owns a policy configuration and informations
 * All data is stored in mc_rtc joint order.
 */
struct PolicyConfig
{
  std::string name;
  std::string folder;
  std::string onnxPath;
  std::string policyYamlPath;
  std::string observationsYamlPath;

  bool useQP = true;
  double policyStepSize = 0.02;
  double kpScale = 1.0;
  double kdScale = 1.0;

  /** @brief Optional joint-group selector from policy.yaml/action/joints. */
  std::string actionJointGroup;

  /** @brief Optional per-joint action scale overrides. Missing entries will use scale 1.0. */
  std::vector<double> actionScale;

  /** @brief Optional er-joint default target pose overrides. No missing entries allowed. */
  std::vector<double> defaultPosition;

  /** @brief Mandatory kp and kd */
  std::vector<double> kp;
  std::vector<double> kd;

  mc_rtc::Configuration policyConfiguration;
  mc_rtc::Configuration observationsConfiguration;

  static PolicyConfig load(const std::string & policyFolder, std::vector<std::string> mcRtcJoints);

  void validate() const;
};

/** @brief Handles the loadable policies */
class PolicyManager
{
public:
  void load(const mc_rtc::Configuration & controllerConfig, std::vector<std::string> mcRtcJoints);

  bool empty() const { return policies_.empty(); };

  size_t size() const { return policies_.size(); };

  const PolicyConfig & current() const { return get(currentName_); };

  const std::string & currentName() const { return currentName_; };

  const PolicyConfig & get(const std::string & name) const;

  std::vector<std::string> names() const {return orderedNames_; };

  void select(const std::string & name);
  void selectNext();

private:
  std::vector<std::string> orderedNames_;
  std::map<std::string, PolicyConfig> policies_;
  std::string currentName_;
};

} // namespace rlqp
