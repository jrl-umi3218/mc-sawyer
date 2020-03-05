#include <mc_rbdyn/RobotLoader.h>

int main()
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/.."});
  auto robot = mc_rbdyn::RobotLoader::get_robot_module("@MODULE_NAME@");
  std::cout << "@MODULE_NAME@ has " << robot->mb.nrDof() << " dof\n";
  return 0;
}
