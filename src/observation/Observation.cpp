#include "observation/Observation.h"
#include "observation/Observations.h"
#include "ConfigurationHelpers.h"

#include <algorithm>
#include <set>
#include <sstream>

#include <mc_rtc/logging.h>

namespace rlqp
{

namespace
{
std::string joinStrings(const std::vector<std::string> & values)
{
  std::ostringstream os;

  for(size_t i = 0; i < values.size(); ++i)
  {
    os << values[i];
    if(i + 1 < values.size())
    {
      os << ", ";
    }
  }

  return os.str();
}

template<typename T>
std::shared_ptr<Observation> makeObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
{
  return std::shared_ptr<Observation>(new T(config, convention));
}

} // namespace

ObservationConvention ObservationConvention::fromConfig(const mc_rtc::Configuration & controllerConfig,
                                                        const std::string & conventionName)
{
  ObservationConvention out;
  out.name = conventionName;

  mc_rtc::Configuration conventionRoot = config::loadConventionRoot(controllerConfig);

  if(!conventionRoot.has("conventions"))
  {
    mc_rtc::log::error_and_throw(
      "[ObservationConvention] No 'conventions' block found. Expected either controller config conventions or policies/conventions.yaml");
  }

  mc_rtc::Configuration conventions = conventionRoot("conventions");

  if(!conventions.has(conventionName))
  {
    mc_rtc::log::error_and_throw(
      "[ObservationConvention] Requested convention '{}' does not exist",
      conventionName);
  }

  mc_rtc::Configuration cfg = conventions(conventionName);

  if(cfg.has("joint_groups"))
  {
    mc_rtc::Configuration groups = cfg("joint_groups");
    std::vector<std::string> keys = groups.keys();

    for(size_t i = 0; i < keys.size(); ++i)
    {
      out.jointGroups[keys[i]] = groups(keys[i], std::vector<std::string>());
    }
  }

  // Load mc_rtc joint groups for cross-convention subset resolution
  if(conventions.has("mc_rtc"))
  {
    mc_rtc::Configuration mcRtcCfg = conventions("mc_rtc");
    if(mcRtcCfg.has("joint_groups"))
    {
      mc_rtc::Configuration mcGroups = mcRtcCfg("joint_groups");
      std::vector<std::string> mcKeys = mcGroups.keys();
      for(size_t i = 0; i < mcKeys.size(); ++i)
      {
        out.mcRtcJointGroups[mcKeys[i]] = mcGroups(mcKeys[i], std::vector<std::string>());
      }
    }
  }

  if(cfg.has("observation_defaults"))
  {
    mc_rtc::Configuration defaults = cfg("observation_defaults");
    std::vector<std::string> keys = defaults.keys();

    for(size_t i = 0; i < keys.size(); ++i)
    {
      out.defaultParameters[keys[i]] = defaults(keys[i]);
    }
  }

  if(cfg.has("type_aliases"))
  {
    mc_rtc::Configuration aliases = cfg("type_aliases");
    std::vector<std::string> keys = aliases.keys();

    for(size_t i = 0; i < keys.size(); ++i)
    {
      std::string alias = keys[i];
      aliases(keys[i], alias);
      out.typeAliases[keys[i]] = alias;
    }
  }

  return out;
}

std::string ObservationConvention::resolveType(const std::string & requestedType) const
{
  std::map<std::string, std::string>::const_iterator it = typeAliases.find(requestedType);

  if(it == typeAliases.end())
  {
    return requestedType;
  }

  return it->second;
}

std::vector<int> ObservationConvention::resolveJointControllerIndices(
  const mc_rtc::Configuration & parameters,
  const std::vector<std::string> & controllerJointOrder,
  const std::vector<int> & fallbackIndices) const
{
  if(!parameters.has("joints"))
  {
    return fallbackIndices;
  }

  // Helper: map a joint name to its index in controllerJointOrder
  auto nameToIdx = [&](const std::string & name) -> int
  {
    std::vector<std::string>::const_iterator it =
      std::find(controllerJointOrder.begin(), controllerJointOrder.end(), name);
    if(it == controllerJointOrder.end())
    {
      mc_rtc::log::error_and_throw(
        "[ObservationConvention] Joint '{}' not found in controllerJointOrder", name);
    }
    return static_cast<int>(std::distance(controllerJointOrder.begin(), it));
  };

  // Try to parse as a group name (string)
  try
  {
    const std::string groupName = parameters("joints", std::string(""));
    if(!groupName.empty())
    {
      // Check RL convention groups (e.g. mjlab.joint_groups.all)
      std::map<std::string, std::vector<std::string> >::const_iterator rlIt =
        jointGroups.find(groupName);
      if(rlIt != jointGroups.end())
      {
        std::vector<int> out;
        out.reserve(rlIt->second.size());
        for(size_t i = 0; i < rlIt->second.size(); ++i)
        {
          out.push_back(nameToIdx(rlIt->second[i]));
        }
        return out;
      }

      // Check mc_rtc groups (e.g. mc_rtc.joint_groups.legs);
      //    return joints filtered to those in the subset, in RL convention "all" order
      std::map<std::string, std::vector<std::string> >::const_iterator mcIt =
        mcRtcJointGroups.find(groupName);
      if(mcIt != mcRtcJointGroups.end())
      {
        const std::vector<std::string> & mcJoints = mcIt->second;
        std::set<std::string> requested(mcJoints.begin(), mcJoints.end());

        std::vector<int> out;
        std::map<std::string, std::vector<std::string> >::const_iterator allIt =
          jointGroups.find("all");
        if(allIt != jointGroups.end())
        {
          for(size_t i = 0; i < allIt->second.size(); ++i)
          {
            if(requested.count(allIt->second[i]))
            {
              out.push_back(nameToIdx(allIt->second[i]));
            }
          }
        }
        if(!out.empty())
        {
          return out;
        }
      }
    }
  }
  catch(...)
  {
  }

  // Try to parse as an explicit list of joint names
  try
  {
    const std::vector<std::string> names = parameters("joints", std::vector<std::string>());
    if(!names.empty())
    {
      std::vector<int> out;
      out.reserve(names.size());
      for(size_t i = 0; i < names.size(); ++i)
      {
        out.push_back(nameToIdx(names[i]));
      }
      return out;
    }
  }
  catch(...)
  {
  }

  return fallbackIndices;
}

mc_rtc::Configuration ObservationConvention::resolveObservationParameters(
  const std::string & requestedType,
  const std::string & internalType,
  const mc_rtc::Configuration & localParameters) const
{
  mc_rtc::Configuration out;

  std::map<std::string, mc_rtc::Configuration>::const_iterator exactIt = defaultParameters.find(requestedType);
  if(exactIt != defaultParameters.end())
  {
    out = exactIt->second;
  }
  else
  {
    std::map<std::string, mc_rtc::Configuration>::const_iterator internalIt = defaultParameters.find(internalType);
    if(internalIt != defaultParameters.end())
    {
      out = internalIt->second;
    }
  }

  std::vector<std::string> keys = localParameters.keys();

  for(size_t i = 0; i < keys.size(); ++i)
  {
    out.add(keys[i], localParameters(keys[i]));
  }

  return out;
}

Observation::Observation(const ObservationConfig & config, const ObservationConvention & convention)
: config_(config), convention_(convention)
{
  if(config_.name.empty())
  {
    config_.name = config_.requestedType.empty() ? config_.type : config_.requestedType;
  }
}

Observation::~Observation()
{
}

Eigen::VectorXd Observation::readScaleVector(const mc_rtc::Configuration & parameters,
                                             const std::string & key,
                                             int size,
                                             double fallback) const
{
  if(!parameters.has(key))
  {
    return Eigen::VectorXd::Constant(size, fallback);
  }

  try
  {
    const std::vector<double> values = parameters(key, std::vector<double>());
    if(values.size() == static_cast<size_t>(size))
    {
      Eigen::VectorXd out(size);
      for(int i = 0; i < size; ++i)
      {
        out(i) = values[static_cast<size_t>(i)];
      }
      return out;
    }

    if(!values.empty())
    {
      mc_rtc::log::error_and_throw(
        "[Observation:{}] Parameter '{}' has size {}, expected {}",
        name(),
        key,
        values.size(),
        size);
    }
  }
  catch(...)
  {
  }

  double scalar = fallback;
  parameters(key, scalar);
  return Eigen::VectorXd::Constant(size, scalar);
}

void ObservationRegistry::registerType(const std::string & type, ObservationFactory factory)
{
  if(type.empty())
  {
    mc_rtc::log::error_and_throw("[ObservationRegistry] Cannot register an empty observation type");
  }

  if(!factory)
  {
    mc_rtc::log::error_and_throw("[ObservationRegistry] Cannot register null factory for '{}'", type);
  }

  if(factories_.find(type) != factories_.end())
  {
    mc_rtc::log::error_and_throw("[ObservationRegistry] Observation type '{}' is already registered", type);
  }

  factories_[type] = factory;
}

std::shared_ptr<Observation> ObservationRegistry::create(const ObservationConfig & config,
                                                         const ObservationConvention & convention) const
{
  std::map<std::string, ObservationFactory>::const_iterator it = factories_.find(config.type);

  if(it == factories_.end())
  {
    mc_rtc::log::error_and_throw(
      "[ObservationRegistry] Unknown observation type '{}'. Known types are: {}",
      config.type,
      joinStrings(knownTypes()));
  }

  return it->second(config, convention);
}

std::vector<std::string> ObservationRegistry::knownTypes() const
{
  std::vector<std::string> out;

  for(std::map<std::string, ObservationFactory>::const_iterator it = factories_.begin();
      it != factories_.end();
      ++it)
  {
    out.push_back(it->first);
  }

  return out;
}

ObservationRegistry makeDefaultObservationRegistry()
{
  ObservationRegistry registry;

  registry.registerType("joint_pos", &makeObservation<JointPosObservation>);
  registry.registerType("joint_vel", &makeObservation<JointVelObservation>);
  registry.registerType("projected_gravity", &makeObservation<ProjectedGravityObservation>);
  registry.registerType("base_ang_vel", &makeObservation<BaseAngVelObservation>);
  registry.registerType("base_lin_vel", &makeObservation<BaseLinVelObservation>);
  registry.registerType("base_orientation", &makeObservation<BaseOrientationObservation>);
  registry.registerType("phase", &makeObservation<PhaseObservation>);
  registry.registerType("last_action", &makeObservation<LastActionObservation>);
  registry.registerType("command", &makeObservation<CommandObservation>);

  return registry;
}

} // namespace rlqp
