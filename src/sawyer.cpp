#include "sawyer.h"
#include "config.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>

#include <fstream>

namespace mc_robots
{

  SawyerCommonRobotModule::SawyerCommonRobotModule() : RobotModule(SAWYER_DESCRIPTION_PATH, "sawyer")
  {
      _forceSensors.push_back(mc_rbdyn::ForceSensor("EEForceSensor", "right_hand", sva::PTransformd(Eigen::Vector3d(0, 0, 0))));
  }

  void SawyerCommonRobotModule::readUrdf(const std::string & robotName,
				      bool fixed,
				      const std::vector<std::string> & filteredLinks)
  {
    urdf_path = path + "/urdf/" + robotName + ".urdf";
    std::ifstream ifs(urdf_path);
    if(ifs.is_open())
    {
      std::stringstream urdf;
      urdf << ifs.rdbuf();
      mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), fixed, filteredLinks);
      mb = res.mb;
      mbc = res.mbc;
      mbg = res.mbg;
      limits = res.limits;

      _visual = res.visual;
      _collisionTransforms = res.collision_tf;
    }
    else
    {
      LOG_ERROR("Could not open Sawyer model at " << urdf_path)
      LOG_ERROR_AND_THROW(std::runtime_error, "Failed to open Sawyer model")
    }
  }

  void SawyerCommonRobotModule::init()
  {
    _bounds = nominalBounds(limits);
  }

  std::vector<std::map<std::string, std::vector<double>>> SawyerCommonRobotModule::nominalBounds(
      const mc_rbdyn_urdf::Limits & limits) const
  {
    std::vector<std::map<std::string, std::vector<double>>> res(0);
    res.push_back(limits.lower);
    res.push_back(limits.upper);
    {
      auto mvelocity = limits.velocity;
      for(auto & mv : mvelocity)
      {
	for(auto & mvi : mv.second)
	{
	  mvi = -mvi;
	}
      }
      res.push_back(mvelocity);
    }
    res.push_back(limits.velocity);
    {
      auto mtorque = limits.torque;
      for(auto & mt : mtorque)
      {
	for(auto & mti : mt.second)
	{
	  mti = -mti;
	}
      }
      res.push_back(mtorque);
    }
    res.push_back(limits.torque);
    return res;
  }

  SawyerRobotModule::SawyerRobotModule(bool fixed)
  {
    readUrdf("sawyer", fixed, gripperLinks);
    init();
  }

} // namespace mc_robots
