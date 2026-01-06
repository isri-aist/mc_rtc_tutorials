#include "MobileArmController.h"

#include <mc_rbdyn/RobotLoader.h>

MobileArmController::MobileArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
    : mc_control::MCController({rm, mc_rbdyn::RobotLoader::get_robot_module(
                                        "env", "/usr/local/share/mc_dingo",
                                        std::string("dingo"))},
                               dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  mc_rtc::log::success("MobileArmController init done ");
}

bool MobileArmController::run()
{
  return mc_control::MCController::run();
}

void MobileArmController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("MobileArmController", MobileArmController)
