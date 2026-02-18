#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/SurfaceTransformTask.h>                                      // TODO: Tasks and constraints 1
#include <mc_tasks/EndEffectorTask.h>                                           // TODO: Tasks and constraints 2
// #include <mc_tasks/PostureTask.h>
// #include <mc_solver/KinematicsConstraint.h>

#include "api.h"

enum Phase
{
  APPROACH = 0,
  HANDLE,
  OPEN,
  DONE
};

struct MobileArmController_DLLAPI MobileArmController : public mc_control::MCController
{
  public:
    MobileArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

  private:
  //   int count = 0;
    Phase phase_ = APPROACH;                                                    // TODO: setting up the controller logic
    std::shared_ptr<mc_tasks::SurfaceTransformTask> handTask_;
    std::shared_ptr<mc_tasks::EndEffectorTask> dingoEndEffectorTask_;
  //   std::shared_ptr<mc_solver::KinematicsConstraint> dingoDynamics_;
    std::shared_ptr<mc_solver::KinematicsConstraint> doorKinematics_;
    std::shared_ptr<mc_tasks::PostureTask> doorPostureTask_;
};