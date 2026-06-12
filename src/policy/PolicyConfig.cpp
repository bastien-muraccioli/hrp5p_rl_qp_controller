#include "policy/PolicyConfig.h"
#include "ConfigurationHelpers.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

namespace
{
  double find_value(std::map<std::string, double> map, std::string joint, std::string map_name, double def = -1)
  {
    auto it = map.find(joint);
    if (it == map.end())
    {
      mc_rtc::log::warning("[PolicyConfig] Missing entry {} in {}", joint, map_name);
      return def;
    }
    return it->second;
  }
} // namespace
namespace rlqp
{

//============================================================================//
// PolicyConfig
//============================================================================//
PolicyConfig PolicyConfig::load(const std::string & policyFolder, std::vector<std::string> mcRtcJoints)
{
  PolicyConfig out;

  out.folder = policyFolder;
  out.policyYamlPath = config::joinPath(policyFolder, "policy.yaml");
  out.observationsYamlPath = config::joinPath(policyFolder, "observations.yaml");

  out.policyConfiguration.load(out.policyYamlPath);
  out.observationsConfiguration.load(out.observationsYamlPath);

  out.kp = std::vector<double>(mcRtcJoints.size());
  out.kd = std::vector<double>(mcRtcJoints.size());

  out.name = out.policyConfiguration("name", config::basenameWithoutExtension(policyFolder));

  const std::string defaultOnnxName = out.name + ".onnx";
  const std::string onnxFile = out.policyConfiguration("onnx", defaultOnnxName);
  out.onnxPath = config::joinPath(policyFolder, onnxFile);

  std::map<std::string, double> kp_map, kd_map, defaultPos_map, actionScale_map;

  if(out.policyConfiguration.has("control"))
  {
    const mc_rtc::Configuration control = out.policyConfiguration("control");

    out.useQP = control("use_QP", out.policyConfiguration("use_QP", true));
    out.policyStepSize = control("policy_step_size", out.policyConfiguration("policy_step_size", 0.02));
    out.kpScale = control("kp_scale", out.policyConfiguration("pd_gains_ratio", 1.0));
    out.kdScale = control("kd_scale", std::sqrt(out.kpScale));

    kp_map = control("kp", std::map<std::string, double>());
    kd_map = control("kd", std::map<std::string, double>());
    mc_rtc::log::warning("?? {} {} {}", kp_map.size(), kd_map.size(), mcRtcJoints.size());
    if (kp_map.size() < mcRtcJoints.size() || kd_map.size() < mcRtcJoints.size())
      mc_rtc::log::error_and_throw("[PolicyConfig] policy.yaml: kp and kd must contain all joints");
  }
  else
    mc_rtc::log::error_and_throw("[PolicyConfig]: policy.yaml should contain a \"control\" entry");
  if (out.policyConfiguration.has("action"))
  {
    out.policyConfiguration("action")("joints", out.actionJointGroup);
    actionScale_map = out.policyConfiguration("scale", std::map<std::string, double>());
    defaultPos_map = out.policyConfiguration("default_position", std::map<std::string, double>());

    out.actionScale = std::vector<double>(actionScale_map.size());
    out.defaultPosition = std::vector<double>(defaultPos_map.size());
    if (!out.defaultPosition.empty() && out.defaultPosition.size() != mcRtcJoints.size())
      mc_rtc::log::error_and_throw("[PolicyConfig] policy.yaml : default pos should contain all joints if specified");
  }
  else
    mc_rtc::log::error_and_throw("[PolicyConfig]: policy.yaml should contain a \"action\" entry");
    
  for(size_t i = 0; i < mcRtcJoints.size(); ++i)
  {
    const std::string & joint = mcRtcJoints[i];

    out.kp[i] = find_value(kp_map, joint, "kp");
    out.kd[i] = find_value(kd_map, joint, "kd");
    
    if (out.actionScale.size() > 0)
      out.actionScale.push_back(find_value(actionScale_map, joint, "action scale", 1));
    if (out.defaultPosition.size() > 0)
      out.defaultPosition.push_back(find_value(defaultPos_map, joint, "default_position", 0));
  }

  out.validate();
  return out;
}

void PolicyConfig::validate() const
{
  if(name.empty())
    mc_rtc::log::error_and_throw("[PolicyConfig] Policy name cannot be empty");

  if(folder.empty())
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] Policy folder cannot be empty", name);

  if(onnxPath.empty())
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] ONNX path cannot be empty", name);

  if(actionJointGroup.empty())
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] action.joints cannot be empty", name);

  if(policyStepSize <= 0.0)
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.policy_step_size must be positive", name);

  if(kpScale <= 0.0)
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.kp_scale must be positive", name);

  if(kdScale <= 0.0)
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.kd_scale must be positive", name);
  
}

//============================================================================//
// PolicyManager
//============================================================================//
void PolicyManager::load(const mc_rtc::Configuration & controllerConfig, std::vector<std::string> mcRtcJoints)
{
  orderedNames_.clear();
  policies_.clear();
  currentName_.clear();

  const std::string policiesRoot = controllerConfig("policies_root", std::string("policies"));
  const std::vector<std::string> folders =
    config::listPolicies(policiesRoot, {"policy.yaml", "observations.yaml"});

  if(folders.empty())
  {
    mc_rtc::log::error_and_throw(
      "[PolicyManager] No policy folders found in '{}'. Expected subdirectories containing policy.yaml and observations.yaml.",
      policiesRoot);
  }

  for(size_t i = 0; i < folders.size(); ++i)
  {
    PolicyConfig policy = PolicyConfig::load(folders[i], mcRtcJoints);

    if(policies_.find(policy.name) != policies_.end())
      mc_rtc::log::error_and_throw("[PolicyManager] Duplicate policy name '{}'", policy.name);

    orderedNames_.push_back(policy.name);
    policies_[policy.name] = policy;
  }

  std::string defaultPolicy = orderedNames_.front();
  controllerConfig("default_policy", defaultPolicy);
  select(defaultPolicy);

  mc_rtc::log::success("[PolicyManager] Loaded {} policies from '{}'. Active policy: {}",
                       policies_.size(),
                       policiesRoot,
                       currentName_);
}


const PolicyConfig & PolicyManager::get(const std::string & name) const
{
  std::map<std::string, PolicyConfig>::const_iterator it = policies_.find(name);

  if(it == policies_.end())
    mc_rtc::log::error_and_throw("[PolicyManager] Unknown policy '{}'", name);

  return it->second;
}

void PolicyManager::select(const std::string & name)
{
  get(name);
  currentName_ = name;
}

void PolicyManager::selectNext()
{
  if(orderedNames_.empty())
    mc_rtc::log::error_and_throw("[PolicyManager] Cannot select next policy: no policies loaded");

  std::vector<std::string>::const_iterator it =
    std::find(orderedNames_.begin(), orderedNames_.end(), currentName_);

  if(it == orderedNames_.end())
  {
    currentName_ = orderedNames_.front();
    return;
  }

  ++it;

  if(it == orderedNames_.end())
    it = orderedNames_.begin();

  currentName_ = *it;
}

} // namespace rlqp
