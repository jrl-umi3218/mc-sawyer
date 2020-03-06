#include "sawyer.h"
#include "config.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

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
    urdf_path = path + "/urdf/" + robotName + "_collisions.urdf";
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


  std::map<std::string, std::pair<std::string, std::string>> SawyerCommonRobotModule::stdCollisionsFiles(
      const rbd::MultiBody & mb) const
  {
    std::map<std::string, std::pair<std::string, std::string>> res;
    res["pedestal"] = std::pair<std::string, std::string>("pedestal", "sawyer_pv/pedestal");
    res["right_arm_base_link"] = std::pair<std::string, std::string>("right_arm_base_link", "sawyer_pv/base");
    res["head"] = std::pair<std::string, std::string>("head", "sawyer_pv/head");
    res["right_l0"] = std::pair<std::string, std::string>("right_l0", "sawyer_mp3/l0");
    res["right_l1"] = std::pair<std::string, std::string>("right_l1", "sawyer_mp3/l1");
    res["right_l2"] = std::pair<std::string, std::string>("right_l2", "sawyer_pv/l2");
    res["right_l3"] = std::pair<std::string, std::string>("right_l3", "sawyer_pv/l3");
    res["right_l4"] = std::pair<std::string, std::string>("right_l4", "sawyer_pv/l4");
    res["right_l5"] = std::pair<std::string, std::string>("right_l5", "sawyer_pv/l5");
    res["right_l6"] = std::pair<std::string, std::string>("right_l6", "sawyer_mp1/l6");
    return res;
  }

  std::map<std::string, std::pair<std::string, std::string>> SawyerCommonRobotModule::getConvexHull(
      const std::map<std::string, std::pair<std::string, std::string>> & files) const
  {
    std::string convexPath = path + "/convex/";
    std::map<std::string, std::pair<std::string, std::string>> res;
    for(const auto & f : files)
    {
      bfs::path fpath(convexPath + f.second.second + "-ch.txt");
      if(bfs::exists(fpath))
      {
        res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
      }
    }
    return res;
  }

  void SawyerCommonRobotModule::init()
  {
    _bounds = nominalBounds(limits);
    auto fileByBodyName = stdCollisionsFiles(mb);
    _convexHull = getConvexHull(fileByBodyName);

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
      mc_rbdyn::Collision("right_l4", "right_l0", 0.05, 0.01, 0.)
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
