#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI SawyerCommonRobotModule : public mc_rbdyn::RobotModule
{
public:
  SawyerCommonRobotModule();

protected:
  void readUrdf(const std::string & robotName, bool fixed, const std::vector<std::string> & filteredLinks);

  using RobotModule::init;

public:
  std::vector<std::string> gripperLinks;
};

struct MC_ROBOTS_DLLAPI SawyerRobotModule : public SawyerCommonRobotModule
{
public:
  SawyerRobotModule(bool fixed);
};

} // namespace mc_robots
