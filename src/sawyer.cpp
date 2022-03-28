#include "sawyer.h"
#include "config.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <RBDyn/parsers/urdf.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <fstream>

namespace mc_robots
{

SawyerCommonRobotModule::SawyerCommonRobotModule() : RobotModule(SAWYER_DESCRIPTION_PATH, "sawyer")
{
  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("EEForceSensor", "right_hand", sva::PTransformd(Eigen::Vector3d(0, 0, 0))));
}

void SawyerCommonRobotModule::readUrdf(const std::string & robotName,
                                       bool fixed,
                                       const std::vector<std::string> & filteredLinks)
{
  urdf_path = path + "/urdf/" + robotName + ".urdf";
  if(!bfs::exists(urdf_path))
  {
    mc_rtc::log::error_and_throw("Could not open Sawyer model at {}", urdf_path);
  }
  init(rbd::parsers::from_urdf_file(urdf_path, fixed, filteredLinks));

  _minimalSelfCollisions = {
      // Collisions arm-pedestal
      mc_rbdyn::Collision("right_l0", "pedestal", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l1", "pedestal", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l2", "pedestal", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l3", "pedestal", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l4", "pedestal", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l5", "pedestal", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l6", "pedestal", 0.05, 0.01, 0.),

      // Collisions arm-arm
      mc_rbdyn::Collision("right_l4", "right_arm_base_link", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l4", "right_l0", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l4", "head", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l5", "right_arm_base_link", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l5", "right_l0", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l5", "right_l1", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l5", "head", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l6", "right_arm_base_link", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l6", "right_l0", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("right_l6", "head", 0.05, 0.01, 0.),
  };

  _commonSelfCollisions = _minimalSelfCollisions;
}

SawyerRobotModule::SawyerRobotModule(bool fixed)
{
  readUrdf("sawyer", fixed, gripperLinks);
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"sawyer"};
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
      mc_rtc::log::error("Sawyer module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
