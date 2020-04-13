#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

#include <mc_rbdyn_urdf/urdf.h>

#include <mc_robots/api.h>

namespace mc_robots
{

  struct MC_ROBOTS_DLLAPI SawyerCommonRobotModule : public mc_rbdyn::RobotModule
  {
    public:
      SawyerCommonRobotModule();

    protected:

      void readUrdf(const std::string & robotName, bool fixed, const std::vector<std::string> & filteredLinks);

      std::vector<std::map<std::string, std::vector<double>>> nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;

      std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(
        const rbd::MultiBody & mb) const;
      std::map<std::string, std::pair<std::string, std::string>> getConvexHull(
        const std::map<std::string, std::pair<std::string, std::string>> & files) const;

      void init();

    public:
      std::vector<std::string> gripperLinks;
      mc_rbdyn_urdf::Limits limits;
  };

  struct MC_ROBOTS_DLLAPI SawyerRobotModule : public SawyerCommonRobotModule
  {
    public:
      SawyerRobotModule(bool fixed);
  };

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"sawyer", "sawyerNoGripper"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("sawyer")
    if(n == "sawyer")
    {
      return new mc_robots::SawyerRobotModule(true);
    }
    else
    {
      LOG_ERROR("Sawyer module Cannot create an object of type " << n)
      return nullptr;
    }
  }
}
